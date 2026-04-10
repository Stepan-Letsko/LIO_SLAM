#!/usr/bin/env python3
import subprocess
import argparse
import os
import sys
import time
import signal
import re
import csv
import psutil
import pandas as pd
import math
import shutil
from pathlib import Path

# ==========================================
# EXPERIMENT CONFIGURATION
# ==========================================
IMU_RATES = [95.0, 90.0, 85.0, 80.0, 75.0, 70.0, 65.0, 60.0, 
             55.0, 50.0, 45.0, 40.0, 35.0, 30.0, 25.0, 20.0, 
             15.0, 10.0]

LIDAR_RATES = [9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.5]

RESULTS_BASE = Path("/mnt/usb/results/frequency_sweep")
BAG_TO_TUM_SCRIPT = Path(__file__).parent / "bag_to_tum.py"
FAST_LIO_LOG_PATH = Path("/root/ros2_ws/src/FAST_LIO_ROS2/Log/fast_lio_time_log.csv")
MASTER_CSV = RESULTS_BASE / "frequency_results_matrix.csv"

class FrequencySweeper:
    def __init__(self, bag_path, config_file_path, raw_imu, raw_lidar, duration=None):
        self.bag_path = Path(bag_path)
        self.config_path = Path(config_file_path)
        self.raw_imu = raw_imu
        self.raw_lidar = raw_lidar
        self.target_duration = duration
        
        # Save the original config content to restore later
        with open(self.config_path, 'r') as file:
            self.original_config_content = file.read()

        RESULTS_BASE.mkdir(parents=True, exist_ok=True)
        
        if not MASTER_CSV.exists():
            with open(MASTER_CSV, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Experiment", "Target_Hz", "Total_Frames",
                    "Closed_Loop_Drift_m", "APE_RMSE_m",
                    "Avg_Latency_ms", "Max_Latency_ms"
                ])

        self.throttle_script = Path(__file__).parent / "simple_throttle.py"
        self._generate_throttle_script()

    def _generate_throttle_script(self):
        with open(self.throttle_script, "w") as f:
            f.write("""#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node

class SimpleThrottle(Node):
    def __init__(self):
        super().__init__('simple_throttle')
        self.in_topic = sys.argv[1]
        self.target_hz = float(sys.argv[2])
        self.out_topic = sys.argv[3]
        self.topic_type_str = sys.argv[4]
        self.min_interval = 1.0 / self.target_hz
        self.next_publish_time = 0.0
        parts = self.topic_type_str.split('/')
        pkg, msg_name = parts[0], parts[-1]
        import importlib
        module = importlib.import_module(f"{pkg}.msg")
        self.msg_class = getattr(module, msg_name)
        qos = 10000
        self.pub = self.create_publisher(self.msg_class, self.out_topic, qos)
        self.sub = self.create_subscription(self.msg_class, self.in_topic, self.callback, qos)
        self.get_logger().info(f"Throttling {self.in_topic} -> {self.out_topic} at {self.target_hz} Hz")

    def callback(self, msg):
        try:
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except AttributeError:
            msg_time = self.get_clock().now().nanoseconds / 1e9
            
        if self.next_publish_time == 0.0:
            self.next_publish_time = msg_time
            
        if msg_time >= self.next_publish_time:
            self.pub.publish(msg)
            self.next_publish_time = max(self.next_publish_time + self.min_interval, msg_time)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
""")
        os.chmod(self.throttle_script, 0o755)

    def update_config(self, imu_topic, lid_topic):
        """Updates the YAML config with dynamically remapped topics."""
        with open(self.config_path, 'r') as file:
            content = file.read()
        
        # Dynamic Topic Injection
        content = re.sub(r'imu_topic:\s*[^\n]+', f'imu_topic: "{imu_topic}"', content)
        content = re.sub(r'lid_topic:\s*[^\n]+', f'lid_topic: "{lid_topic}"', content)

        with open(self.config_path, 'w') as file:
            file.write(content)

    def restore_config(self):
        """Restores the YAML config to its exact original state."""
        with open(self.config_path, 'w') as file:
            file.write(self.original_config_content)
        print(f"\nRestored original configuration to {self.config_path.name}")

    def get_bag_duration(self):
        try:
            res = subprocess.run(['ros2', 'bag', 'info', str(self.bag_path)], capture_output=True, text=True)
            match = re.search(r"Duration:\s+(\d+\.\d+)s", res.stdout)
            return float(match.group(1)) if match else 100.0
        except Exception:
            return 100.0

    def find_mapping_pid(self):
        # Retry for 10 seconds to find the node
        for _ in range(10):
            for proc in psutil.process_iter(['pid', 'cmdline']):
                if proc.info['cmdline'] and 'fastlio_mapping' in ' '.join(proc.info['cmdline']):
                    return proc.info['pid']
            time.sleep(1)
        return None

    def calculate_drift(self, tum_file):
        """Calculates 3D Euclidean drift between the first and last pose (Option 2)."""
        try:
            with open(tum_file, 'r') as f:
                lines = [l.strip() for l in f.readlines() if l.strip() and not l.startswith('#')]
                if len(lines) < 2: return 0.0
                first = lines[0].split()
                last = lines[-1].split()
                
                # TUM format: timestamp tx ty tz qx qy qz qw
                dx = float(last[1]) - float(first[1])
                dy = float(last[2]) - float(first[2])
                dz = float(last[3]) - float(first[3])
                return math.sqrt(dx**2 + dy**2 + dz**2)
        except Exception as e:
            print(f"Error calculating drift: {e}")
            return -1.0

    def evaluate_ape(self, gt_tum, test_tum):
        """Uses evo to compute APE (RMSE) against the pseudo-ground truth (Option 1)."""
        if not Path(gt_tum).exists() or not Path(test_tum).exists():
            return -1.0
        
        cmd = ["evo_ape", "tum", str(gt_tum), str(test_tum), "--align"]
        res = subprocess.run(cmd, capture_output=True, text=True)
        
        # Regex out the RMSE from evo's output table
        match = re.search(r"rmse\s+([\d\.]+)", res.stdout)
        if match:
            return float(match.group(1))
        return -1.0

    def run_iteration(self, experiment_name, target_hz, is_baseline=False):
        iter_name = "Baseline" if is_baseline else f"{experiment_name}_{target_hz}Hz"
        iter_dir = RESULTS_BASE / iter_name
        
        if iter_dir.exists():
            shutil.rmtree(iter_dir)
        iter_dir.mkdir(parents=True)

        print("\n" + "="*50)
        print(f" RUNNING: {iter_name}")
        print("="*50)
        
        # 1. Setup Topics & Throttling
        proc_throttle = None
        active_imu = self.raw_imu
        active_lidar = self.raw_lidar

        if experiment_name == "IMU_Starvation" and not is_baseline:
            active_imu = "/imu/data_throttled"
            print(f"   -> Throttling IMU to {target_hz} Hz...")
            topic_type = "sensor_msgs/msg/Imu"
            cmd = ['python3', str(self.throttle_script), self.raw_imu, str(target_hz), active_imu, topic_type, '--ros-args', '-p', 'use_sim_time:=true']
            proc_throttle = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
        elif experiment_name == "LiDAR_Starvation" and not is_baseline:
            active_lidar = "/lidar_throttled"
            print(f"   -> Throttling LiDAR to {target_hz} Hz...")
            topic_type = "sensor_msgs/msg/PointCloud2"
            cmd = ['python3', str(self.throttle_script), self.raw_lidar, str(target_hz), active_lidar, topic_type, '--ros-args', '-p', 'use_sim_time:=true']
            proc_throttle = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 2. Inject Configs
        self.update_config(imu_topic=active_imu, lid_topic=active_lidar)

        # Cleanup old C++ timing log to prevent cross-contamination
        if FAST_LIO_LOG_PATH.exists():
            FAST_LIO_LOG_PATH.unlink()

        # 3. Start FAST-LIO2
        print("   -> Launching FAST-LIO2...")
        log_file = open(iter_dir / "process_log.txt", "w")
        launch_cmd = ['ros2', 'launch', 'fast_lio', 'mapping.launch.py', 
                      f'config_file:={self.config_path.name}', 'rviz:=false']
        proc_mapping = subprocess.Popen(launch_cmd, stdout=log_file, stderr=subprocess.STDOUT)
        mapping_pid = self.find_mapping_pid()

        # 4. Start Odometry Recorder
        bag_out = iter_dir / "odom_record"
        print(f"   -> Recording Odometry to {bag_out.name}...")
        rec_cmd = ['ros2', 'bag', 'record', '/Odometry', '-o', str(bag_out)]
        proc_rec = subprocess.Popen(rec_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Wait for nodes to link up
        time.sleep(3)

        # 5. Play Dataset
        bag_duration = self.get_bag_duration()
        actual_duration = self.target_duration if self.target_duration and self.target_duration < bag_duration else bag_duration
        print(f"   -> Playing Bag ({actual_duration:.1f}s)...")
        play_cmd = ['ros2', 'bag', 'play', str(self.bag_path), '--clock']
        proc_play = subprocess.Popen(play_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        start_t = time.time()
        try:
            while proc_play.poll() is None:
                elapsed = time.time() - start_t
                percent = min(100, (elapsed / actual_duration) * 100)
                bar = "█" * int(percent // 2) + "-" * (50 - int(percent // 2))
                sys.stdout.write(f"\r|{bar}| {percent:.1f}% ")
                sys.stdout.flush()
                
                if self.target_duration and elapsed >= self.target_duration:
                    break
                    
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n Interrupted by user.")
            sys.exit(1)
        finally:
            sys.stdout.write(f"\r|{'█' * 50}| 100.0% \n")
            print("   -> Shutting down and saving data...")
            
            if proc_play.poll() is None: os.kill(proc_play.pid, signal.SIGINT)
            if mapping_pid:
                os.kill(mapping_pid, signal.SIGINT)
            os.kill(proc_rec.pid, signal.SIGINT)
            os.kill(proc_mapping.pid, signal.SIGINT)
            if proc_throttle: os.kill(proc_throttle.pid, signal.SIGINT)
            
            # Wait for graceful shutdown & log dump
            proc_mapping.wait()
            log_file.close()
            time.sleep(3)

        # 6. Extract TUM and Compute Metrics
        tum_out = iter_dir / f"{iter_name}.tum"
        print("   -> Extracting TUM trajectory...")
        subprocess.run(['python3', str(BAG_TO_TUM_SCRIPT), str(bag_out), str(tum_out)], 
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Clean up recorded bag to save storage space
        if bag_out.exists():
            print("   -> Removing intermediate rosbag to save storage...")
            shutil.rmtree(bag_out)

        # Check frames
        total_frames = 0
        if tum_out.exists():
            with open(tum_out, 'r') as f:
                total_frames = sum(1 for line in f if not line.startswith('#'))

        # Compute Start-to-End Drift (Option 2)
        drift_m = self.calculate_drift(tum_out)

        # Compute APE against baseline (Option 1)
        rmse_m = 0.0
        if is_baseline:
            # Save baseline globally for future reference
            shutil.copy(tum_out, RESULTS_BASE / "baseline_gt.tum")
        else:
            print("   -> Running Evo APE evaluation...")
            rmse_m = self.evaluate_ape(RESULTS_BASE / "baseline_gt.tum", tum_out)

        # Extract Timing Metrics & Plot
        avg_lat, max_lat = 0.0, 0.0
        if FAST_LIO_LOG_PATH.exists():
            dest_csv = iter_dir / "fast_lio_time_log.csv"
            shutil.copy(FAST_LIO_LOG_PATH, dest_csv)
            
            plot_script = Path(__file__).parent / "plot_latency.py"
            if plot_script.exists():
                subprocess.run(['python3', str(plot_script), str(dest_csv), str(iter_dir / "latency_plot.png")], stdout=subprocess.DEVNULL)
                
            try:
                df = pd.read_csv(FAST_LIO_LOG_PATH, skipinitialspace=True)
                if 'math_time' in df.columns:
                    lat_data = df['math_time']
                    if 'io_time' in df.columns:
                        lat_data += df['io_time']
                    avg_lat = lat_data.mean() * 1000.0
                    max_lat = lat_data.max() * 1000.0
            except Exception as e:
                print(f"   -> Failed to parse timing log: {e}")

        print(f"   -> Result: {total_frames} Frames | Drift: {drift_m:.3f}m | RMSE: {rmse_m:.3f}m | Avg Lat: {avg_lat:.2f}ms")

        # Append to master log
        with open(MASTER_CSV, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                experiment_name if not is_baseline else "Baseline",
                target_hz if not is_baseline else "MAX",
                total_frames,
                f"{drift_m:.4f}",
                f"{rmse_m:.4f}" if not is_baseline else "0.0",
                f"{avg_lat:.2f}",
                f"{max_lat:.2f}"
            ])
        return tum_out

    def run_all(self):
        print(f"\n--- BEGINNING ALGORITHMIC SENSOR STARVATION EXPERIMENT ---")
        
        # Phase 0: Golden Trajectory
        self.run_iteration("Baseline", target_hz=None, is_baseline=True)
        
        # Phase A: IMU Starvation
        for hz in IMU_RATES:
            self.run_iteration("IMU_Starvation", hz)
            
        # Phase B: LiDAR Starvation
        for hz in LIDAR_RATES:
            self.run_iteration("LiDAR_Starvation", hz)
            
        print(f"\nALL EXPERIMENTS COMPLETE! Results saved to {MASTER_CSV}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FAST-LIO2 Sensor Frequency Robustness Analysis")
    parser.add_argument("bag_path", help="Path to input rosbag")
    parser.add_argument("--config", default="/root/ros2_ws/src/FAST_LIO_ROS2/config/velodyne.yaml")
    parser.add_argument("--imu_topic", default="/imu/data", help="Original IMU topic in the bag")
    parser.add_argument("--lidar_topic", default="/velodyne_points", help="Original LiDAR topic in the bag")
    parser.add_argument("--duration", type=float, default=None, help="Only process the first N seconds of the bag")
    
    args = parser.parse_args()
    sweeper = FrequencySweeper(args.bag_path, args.config, args.imu_topic, args.lidar_topic, args.duration)
    try:
        sweeper.run_all()
    finally:
        sweeper.restore_config()