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
import matplotlib
matplotlib.use('Agg') # Use headless backend to prevent X11 display errors
import matplotlib.pyplot as plt
import shutil
from pathlib import Path

# ==========================================
# EXPERIMENT CONFIGURATION
# ==========================================
K_VALUES = [1, 2, 3, 4, 5]
V_VALUES = [0.2, 0.4, 0.6, 0.8, 1.0]

RESULTS_BASE = Path("/root/ros2_ws/src/results/parameter_sweep")
FAST_LIO_LOG_PATH = Path("/root/ros2_ws/src/FAST_LIO_ROS2/Log/fast_lio_time_log.csv")
MASTER_CSV = RESULTS_BASE / "sweep_results_matrix.csv"

class ParameterSweeper:
    def __init__(self, bag_path, config_file_path, duration=None):
        self.bag_path = Path(bag_path)
        self.config_path = Path(config_file_path)
        self.target_duration = duration
        
        # Create Directory
        RESULTS_BASE.mkdir(parents=True, exist_ok=True)
        
        # Initialize CSV Header if it doesn't exist
        if not MASTER_CSV.exists():
            with open(MASTER_CSV, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "k_stride", "v_size", "total_frames", 
                    "avg_latency_ms", "max_latency_ms", 
                    "peak_ram_mb", "avg_cpu_percent", "peak_cpu_percent"
                ])

    def update_config(self, k, v):
        """Uses Regex to update the YAML so we don't lose the comments/formatting"""
        with open(self.config_path, 'r') as file:
            content = file.read()

        content = re.sub(r'point_filter_num:\s*\d+', f'point_filter_num: {k}', content)
        content = re.sub(r'filter_size_surf:\s*[\d\.]+', f'filter_size_surf: {v}', content)
        content = re.sub(r'filter_size_map:\s*[\d\.]+', f'filter_size_map: {v}', content)

        with open(self.config_path, 'w') as file:
            file.write(content)

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

    def run_sweep(self):
        bag_duration = self.get_bag_duration()
        actual_duration = self.target_duration if self.target_duration and self.target_duration < bag_duration else bag_duration
        print(f"\n Starting 2D Parameter Sweep on {self.bag_path.name} (Running for {actual_duration:.1f}s)")
        print(f"Matrix: K={K_VALUES} | V={V_VALUES}")
        print(f"Output: {MASTER_CSV}\n")

        for k in K_VALUES:
            for v in V_VALUES:
                iter_dir = RESULTS_BASE / f"k_{k}_v_{v}"
                iter_dir.mkdir(parents=True, exist_ok=True)

                print("="*50)
                print(f" RUNNING ITERATION: k={k}, v={v}")
                print("="*50)
                
                # 1. Inject Parameters
                self.update_config(k, v)
                
                # 2. Cleanup old C++ log to prevent cross-contamination
                if FAST_LIO_LOG_PATH.exists():
                    FAST_LIO_LOG_PATH.unlink()

                # 3. Launch FAST-LIO Headless (No Rviz, No stdout buffering)
                print("   -> Launching FAST-LIO2...")
                log_file = open(iter_dir / "process_log.txt", "w")
                launch_cmd = ['ros2', 'launch', 'fast_lio', 'mapping.launch.py', 
                              f'config_file:={self.config_path.name}', 'rviz:=false']
                proc_mapping = subprocess.Popen(launch_cmd, stdout=log_file, stderr=subprocess.STDOUT)
                
                mapping_pid = self.find_mapping_pid()
                if not mapping_pid:
                    print("   ->  ERROR: Could not find FAST-LIO PID. Skipping...")
                    proc_mapping.kill()
                    log_file.close()
                    continue

                # 4. Play Bag File
                print("   -> Waiting 3s for init, then playing bag...")
                time.sleep(3)
                play_cmd = ['ros2', 'bag', 'play', str(self.bag_path), '--clock']
                proc_play = subprocess.Popen(play_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                # 5. Resource Monitoring Loop
                proc = psutil.Process(mapping_pid)
                start_t = time.time()
                resource_stats = []
                
                try:
                    while proc_play.poll() is None: # While bag is playing
                        elapsed = time.time() - start_t
                        percent = min(100, (elapsed / actual_duration) * 100)
                        
                        try:
                            cpu = proc.cpu_percent(interval=None)
                            ram = proc.memory_info().rss / (1024 * 1024) # MB
                            resource_stats.append((elapsed, cpu, ram))
                        except psutil.NoSuchProcess:
                            break
                        
                        bar = "█" * int(percent // 2) + "-" * (50 - int(percent // 2))
                        sys.stdout.write(f"\r|{bar}| {percent:.1f}% | CPU: {cpu:.1f}% | RAM: {ram:.1f} MB")
                        sys.stdout.flush()

                        # Check if we hit the target duration
                        if self.target_duration and elapsed >= self.target_duration:
                            sys.stdout.write(f"\r|{'█' * 50}| 100.0% | Reached target duration ({self.target_duration}s)\n")
                            sys.stdout.flush()
                            os.kill(proc_play.pid, signal.SIGINT)
                            break

                        time.sleep(0.5)
                        
                except KeyboardInterrupt:
                    print("\n Interrupted by user. Aborting sweep.")
                    os.kill(proc_play.pid, signal.SIGINT)
                    if mapping_pid:
                        os.kill(mapping_pid, signal.SIGINT)
                    os.kill(proc_mapping.pid, signal.SIGINT)
                    sys.exit(1)

                print("\n   -> Playback Finished. Shutting down node...")
                if mapping_pid:
                    os.kill(mapping_pid, signal.SIGINT) # Kill the C++ node directly to prevent zombies
                os.kill(proc_mapping.pid, signal.SIGINT) # Signal the launch process
                proc_mapping.wait() # Wait for graceful shutdown & log dump
                log_file.close()

                # 6. Extract Data & Append to Matrix
                peak_cpu = max([s[1] for s in resource_stats]) if resource_stats else 0.0
                avg_cpu = sum([s[1] for s in resource_stats]) / len(resource_stats) if resource_stats else 0.0
                peak_ram = max([s[2] for s in resource_stats]) if resource_stats else 0.0
                
                # Save detailed resource logs and plots to iteration subfolder
                if resource_stats:
                    df_res = pd.DataFrame(resource_stats, columns=['Time', 'CPU', 'RAM'])
                    df_res.to_csv(iter_dir / "resources.csv", index=False)
                    
                    fig, ax1 = plt.subplots(figsize=(10, 6))
                    ax2 = ax1.twinx()
                    ax1.plot(df_res['Time'], df_res['CPU'], 'g-', alpha=0.6, label='CPU %')
                    ax2.plot(df_res['Time'], df_res['RAM'], 'b-', linewidth=2, label='RAM (MB)')
                    ax1.set_ylabel('CPU (%)', color='g')
                    ax2.set_ylabel('RAM (MB)', color='b')
                    ax1.set_xlabel('Time (s)')
                    plt.title(f"Resource Usage: k={k}, v={v}")
                    plt.grid(True, alpha=0.3)
                    plt.savefig(iter_dir / "resource_plot.png")
                    plt.close() # Prevent memory leaks in loops

                total_frames, avg_lat, max_lat = 0, 0.0, 0.0
                if FAST_LIO_LOG_PATH.exists():
                    # Copy detailed latency log and generate its plot
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
                            total_frames = len(lat_data)
                            avg_lat = lat_data.mean() * 1000.0 # Convert to ms
                            max_lat = lat_data.max() * 1000.0
                    except Exception as e:
                        print(f"   ->  Failed to parse timing log: {e}")
                
                print(f"   -> Results: {total_frames} frames | {avg_lat:.2f} ms avg lat | {peak_cpu:.1f}% peak CPU")
                
                with open(MASTER_CSV, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        k, v, total_frames, 
                        f"{avg_lat:.2f}", f"{max_lat:.2f}", 
                        f"{peak_ram:.2f}", f"{avg_cpu:.2f}", f"{peak_cpu:.2f}"
                    ])

        print(f"\n SWEEP COMPLETE! All data saved to: {MASTER_CSV}")

        # Automatically generate the heatmaps after the sweep finishes
        plot_script = Path(__file__).parent / "plot_sweep_heatmaps.py"
        if plot_script.exists():
            print("\n   -> Generating summary heatmaps...")
            subprocess.run(['python3', str(plot_script), str(MASTER_CSV)])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Automate FAST-LIO2 Parameter Sweep")
    parser.add_argument("bag_path", help="Path to input rosbag")
    parser.add_argument("--config", default="/root/ros2_ws/src/FAST_LIO_ROS2/config/velodyne.yaml", 
                        help="Absolute path to the yaml config to modify")
    parser.add_argument("--duration", type=float, default=None, 
                        help="Only process the first N seconds of the bag")
    
    args = parser.parse_args()
    
    if "ROS_DISTRO" not in os.environ:
        print("WARNING: ROS 2 environment not sourced! Please run 'source /opt/ros/humble/setup.bash' first.")
        sys.exit(1)

    sweeper = ParameterSweeper(args.bag_path, args.config, args.duration)
    sweeper.run_sweep()
