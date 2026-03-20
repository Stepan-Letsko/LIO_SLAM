import subprocess
import argparse
import os
import sys
import time
import signal
import re
import csv
import threading
import psutil
import pandas as pd
import matplotlib.pyplot as plt
import shutil
from pathlib import Path

# ==========================================
# CONFIGURATION
# ==========================================
RESULTS_BASE = Path("/root/ros2_ws/src/results/full_analysis_results")
# The map name defined in your yaml (usually ./scans.pcd or ./RAM_TEST.pcd)
EXPECTED_PCD_NAME = "Current_map.pcd" 
FAST_LIO_LOG_PATH = Path("/root/ros2_ws/src/FAST_LIO_ROS2/Log/fast_lio_time_log.csv")
BAG_TO_TUM_SCRIPT = Path(__file__).parent / "bag_to_tum.py"
BAG_TO_PCD_SCRIPT = Path(__file__).parent / "bag_to_pcd.py"

class FastLioAnalyzer:
    def __init__(self, bag_path, config_file, use_decoder=False, start_offset=0.0, duration=None):
        self.bag_path = Path(bag_path)
        self.bag_name = self.bag_path.stem
        self.config_file = config_file
        self.use_decoder = use_decoder
        self.start_offset = start_offset
        self.duration = duration
        self.output_dir = RESULTS_BASE / f"{self.bag_name}_FULL_ANALYSIS"
        
        # Data Containers
        self.latencies = []
        self.resource_stats = []
        self.mapping_pid = None
        self.stop_event = threading.Event()
        self.total_duration = 0
        
        # Create Directory
        if self.output_dir.exists():
            shutil.rmtree(self.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def get_bag_duration(self):
        try:
            res = subprocess.run(['ros2', 'bag', 'info', str(self.bag_path)], capture_output=True, text=True)
            match = re.search(r"Duration:\s+(\d+\.\d+)s", res.stdout)
            return float(match.group(1)) if match else 100.0
        except:
            return 100.0

    def find_mapping_pid(self):
        # Retry for 10 seconds to find the node
        for _ in range(10):
            for proc in psutil.process_iter(['pid', 'cmdline']):
                if proc.info['cmdline'] and 'fastlio_mapping' in ' '.join(proc.info['cmdline']):
                    return proc.info['pid']
            time.sleep(1)
        return None

    def task_log_parser(self, process):
        """Thread 1: Reads stdout line-by-line for Latency metrics"""
        log_path = self.output_dir / "process_log.txt"
        csv_path = self.output_dir / "latency_data.csv"
        
        with open(log_path, "w") as f_log, open(csv_path, "w", newline="") as f_csv:
            writer = csv.writer(f_csv)
            writer.writerow(["match", "solve", "ICP", "total"]) # Simplified header
            
            # Regex for the timing line
            pattern = r"match:\s*([\d\.]+).*solve:\s*([\d\.]+).*ICP:\s*([\d\.]+).*total:\s*([\d\.]+)"
            
            while not self.stop_event.is_set():
                line = process.stdout.readline()
                if not line: break
                
                f_log.write(line) # Save full log
                
                if "ave total:" in line:
                    match = re.search(pattern, line)
                    if match:
                        vals = [float(x) for x in match.groups()]
                        writer.writerow(vals)
                        self.latencies.append(vals[3]) # Index 3 is total time

    def task_resource_monitor(self):
        """Thread 2: Polls CPU/RAM every 0.5s"""
        while self.mapping_pid is None and not self.stop_event.is_set():
            time.sleep(0.5)
            
        if not self.mapping_pid: return

        try:
            proc = psutil.Process(self.mapping_pid)
            start_t = time.time()
            
            while not self.stop_event.is_set():
                try:
                    cpu = proc.cpu_percent(interval=None)
                    ram = proc.memory_info().rss / (1024 * 1024) # MB
                    elapsed = time.time() - start_t
                    self.resource_stats.append([elapsed, cpu, ram])
                except:
                    break # Process died
                time.sleep(0.5)
        except psutil.NoSuchProcess:
            return

    def generate_report(self):
        print("\n\n Generating Final Report...")
        
        # 1. Resource Plot
        if self.resource_stats:
            df_res = pd.DataFrame(self.resource_stats, columns=['Time', 'CPU', 'RAM'])
            df_res.to_csv(self.output_dir / "resources.csv", index=False)
            
            fig, ax1 = plt.subplots(figsize=(10, 6))
            ax2 = ax1.twinx()
            ax1.plot(df_res['Time'], df_res['CPU'], 'g-', alpha=0.6, label='CPU %')
            ax2.plot(df_res['Time'], df_res['RAM'], 'b-', linewidth=2, label='RAM (MB)')
            
            ax1.set_ylabel('CPU (%)', color='g')
            ax2.set_ylabel('RAM (MB)', color='b')
            ax1.set_xlabel('Time (s)')
            plt.title(f"Resource Usage: {self.bag_name}")
            plt.grid(True, alpha=0.3)
            plt.savefig(self.output_dir / "resource_plot.png")
            
            peak_ram = df_res['RAM'].max()
            avg_cpu = df_res['CPU'].mean()
            peak_cpu = df_res['CPU'].max()
        else:
            peak_ram = 0
            avg_cpu = 0
            peak_cpu = 0

        # 2. Latency Stats (Prefer C++ Log if available)
        cpp_log_path = self.output_dir / "fast_lio_time_log.csv"
        source_type = "STDOUT (Approximate)"
        
        if cpp_log_path.exists():
            try:
                df = pd.read_csv(cpp_log_path, skipinitialspace=True)
                # 'math_time' is usually the total processing time in the C++ log
                if 'math_time' in df.columns:
                    lat_data = df['math_time']
                    if 'io_time' in df.columns:
                        lat_data = lat_data + df['io_time']
                    total_frames = len(lat_data)
                    avg_lat = lat_data.mean()
                    max_lat = lat_data.max()
                    source_type = "C++ LOG (Accurate)"
                else:
                    # Fallback to stdout data
                    total_frames = len(self.latencies)
                    avg_lat = sum(self.latencies)/total_frames if total_frames > 0 else 0
                    max_lat = max(self.latencies) if total_frames > 0 else 0
            except:
                pass
        else:
            total_frames = len(self.latencies)
            avg_lat = sum(self.latencies)/total_frames if total_frames > 0 else 0
            max_lat = max(self.latencies) if total_frames > 0 else 0
        
        summary = (
            f"========================================\n"
            f" FINAL RESULTS: {self.bag_name}\n"
            f"========================================\n"
            f" Total Processed Frames: {total_frames}\n"
            f" Avg Processing Time:    {avg_lat*1000:.2f} ms\n"
            f" Max Processing Time:    {max_lat*1000:.2f} ms\n"
            f" Data Source:            {source_type}\n"
            f"----------------------------------------\n"
            f" Peak CPU Usage:         {peak_cpu:.2f} %\n"
            f" Peak RAM Usage:         {peak_ram:.2f} MB\n"
            f" Avg CPU Usage:          {avg_cpu:.2f} %\n"
            f"========================================\n"
        )
        
        print(summary)
        with open(self.output_dir / "summary.txt", "w") as f:
            f.write(summary)
            
        self.update_global_history(total_frames, avg_lat, max_lat, peak_ram, peak_cpu, avg_cpu)

    def update_global_history(self, frames, avg_lat, max_lat, peak_ram, peak_cpu, avg_cpu):
        """Appends the results of this run to a master CSV for easy comparison."""
        master_csv = RESULTS_BASE / "benchmark_comparison.csv"
        file_exists = master_csv.exists()
        
        try:
            with open(master_csv, "a", newline="") as f:
                writer = csv.writer(f)
                # Write Header if new file
                if not file_exists:
                    writer.writerow(["Timestamp", "Bag Name", "Config", "Frames", "Avg Latency (ms)", "Max Latency (ms)", "Peak RAM (MB)", "Peak CPU (%)", "Avg CPU (%)"])
                
                # Write Data
                writer.writerow([
                    time.strftime("%Y-%m-%d %H:%M:%S"),
                    self.bag_name,
                    self.config_file,
                    frames,
                    f"{avg_lat*1000:.2f}",
                    f"{max_lat*1000:.2f}",
                    f"{peak_ram:.2f}",
                    f"{peak_cpu:.2f}",
                    f"{avg_cpu:.2f}"
                ])
            print(f"   -> Added entry to master comparison log: {master_csv}")
        except Exception as e:
            print(f"   -> Warning: Could not update master log: {e}")

    def run(self):
        self.total_duration = self.get_bag_duration()
        print(f"Starting Full Analysis for {self.bag_name} ({self.total_duration:.1f}s)")
        print(f"Output: {self.output_dir}")

        # 0. Start Decoder if requested (For DARPA/Raw Packets)
        proc_decoder = None
        if self.use_decoder:
            print("   -> Launching Velodyne Decoder...")
            decoder_cmd = ['ros2', 'launch', 'fast_lio', 'decoder.launch.py']
            proc_decoder = subprocess.Popen(decoder_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(2) # Give it a moment to initialize

        # 1. Start Recording (Background) - Record Odometry, Cloud, and Path
        bag_out = self.output_dir / "recorded_bag"
        print("   -> Starting Recorder...")
        rec_cmd = ['ros2', 'bag', 'record', '/Odometry', '/cloud_registered', '/path', '-o', str(bag_out)]
        proc_rec = subprocess.Popen(rec_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 2. Start FAST-LIO (Unbuffered)
        print(f"   -> Launching Node ({self.config_file})...")
        # stdbuf -oL forces line buffering so we can read logs instantly
        launch_cmd = ['stdbuf', '-oL', 'ros2', 'launch', 'fast_lio', 'mapping.launch.py', 
                      f'config_file:={self.config_file}', 
                      'rviz:=false'
                      ]
        proc_mapping = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        
        # 3. Find PID
        self.mapping_pid = self.find_mapping_pid()
        
        # 4. Start Monitoring Threads
        t_log = threading.Thread(target=self.task_log_parser, args=(proc_mapping,))
        t_res = threading.Thread(target=self.task_resource_monitor)
        t_log.start()
        t_res.start()

        # 5. Start Playback
        print("Waiting 5 seconds for node to initialise...")
        time.sleep(5) 
        print("   -> Playing Bag...")
        
        # Construct Bag Play Command
        play_cmd = ['ros2', 'bag', 'play', str(self.bag_path), '--clock']
        if self.start_offset > 0:
            play_cmd.extend(['--start-offset', str(self.start_offset)])
        
        # Note: We handle duration manually in the loop to ensure clean shutdown
        proc_play = subprocess.Popen(play_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 6. Progress Bar Loop
        start_t = time.time()
        # If user specified a duration, use that. Otherwise use bag length.
        target_duration = self.duration if self.duration else self.total_duration

        try:
            while proc_play.poll() is None:
                elapsed = time.time() - start_t
                percent = min(100, (elapsed / target_duration) * 100)
                
                # Dynamic Status Line
                ram_str = f"{self.resource_stats[-1][2]:.0f}" if self.resource_stats else "0"
                frames_str = f"{len(self.latencies)}"
                
                bar = "█" * int(percent // 2) + "-" * (50 - int(percent // 2))
                sys.stdout.write(f"\r|{bar}| {percent:.1f}% | RAM: {ram_str} MB | Frames: {frames_str}")
                sys.stdout.flush()
                time.sleep(0.5)
                
                # Manual Duration Check
                if self.duration and elapsed >= self.duration:
                    break

        except KeyboardInterrupt:
            print("\n Interrupted!")

        # 7. Cleanup
        print("\n\n Finishing Up...")
        
        # Trigger Map Save
        print("   -> Triggering Map Save...")
        try:
            subprocess.run(['ros2', 'service', 'call', '/map_save', 'std_srvs/srv/Trigger'], 
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except: pass

        # Stop Threads
        self.stop_event.set()
        
        # Kill Processes
        os.kill(proc_play.pid, signal.SIGINT) if proc_play.poll() is None else None
        os.kill(proc_rec.pid, signal.SIGINT)
        os.kill(proc_mapping.pid, signal.SIGINT)
        if proc_decoder: os.kill(proc_decoder.pid, signal.SIGINT)
        
        # Wait for threads
        t_log.join()
        t_res.join()
        
        # Move Map File
        if Path(EXPECTED_PCD_NAME).exists():
            shutil.move(EXPECTED_PCD_NAME, self.output_dir / "final_map.pcd")
            print("   -> Map Saved successfully.")
        elif BAG_TO_PCD_SCRIPT.exists() and bag_out.exists():
            print("   -> Reconstructing map from recorded bag (RAM Saving Mode)...")
            pcd_out = self.output_dir / "final_map.pcd"
            subprocess.run([
                'python3', str(BAG_TO_PCD_SCRIPT), 
                str(bag_out), 
                '--topic', '/cloud_registered',
                '--output', str(pcd_out),
                '--voxel', '0.05'
            ], check=True)
            print(f"   -> Map reconstructed to {pcd_out.name}")
            
        # Extract Trajectory (TUM format) for Evo
        if BAG_TO_TUM_SCRIPT.exists() and bag_out.exists():
            tum_out = self.output_dir / f"{self.bag_name}_trajectory.tum"
            print(f"   -> Extracting trajectory to {tum_out.name}...")
            subprocess.run(['python3', str(BAG_TO_TUM_SCRIPT), str(bag_out), str(tum_out)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
        # Copy C++ Log CSV and Generate Plot
        if FAST_LIO_LOG_PATH.exists():
            dest_csv = self.output_dir / "fast_lio_time_log.csv"
            shutil.copy(FAST_LIO_LOG_PATH, dest_csv)
            print(f"   -> Copied detailed C++ time log to {dest_csv}")
            
            # Call the separate plotting script
            plot_script = Path(__file__).parent / "plot_latency.py"
            if plot_script.exists():
                plot_out = self.output_dir / "latency_plot.png"
                subprocess.run(['python3', str(plot_script), str(dest_csv), str(plot_out)])

        self.generate_report()
        print(f"DONE. All data in {self.output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Full FAST-LIO Analysis")
    parser.add_argument("bag_path", help="Path to input rosbag")
    parser.add_argument("config_file", nargs="?", default="velodyne.yaml", help="Config file name (default: velodyne.yaml)")
    parser.add_argument("--decoder", action="store_true", help="Launch Velodyne decoder (for raw packet bags like DARPA)")
    parser.add_argument("--start", type=float, default=0.0, help="Start offset in seconds")
    parser.add_argument("--duration", type=float, default=None, help="Duration to run in seconds (optional)")
    
    args = parser.parse_args()
    
    analyzer = FastLioAnalyzer(args.bag_path, args.config_file, args.decoder, args.start, args.duration)
    analyzer.run()
