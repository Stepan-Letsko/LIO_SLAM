import subprocess
import re
import os
import time
import sys
import signal
from pathlib import Path

def run_and_record(bag_path, config="avia.yaml"):
    # 1. Setup Paths
    bag_name = Path(bag_path).stem
    results_base = Path("/mnt/usb/results")
    output_bag_dir = results_base / f"{bag_name}_recorded_output"
    
    if output_bag_dir.exists():
        print(f" Warning: {output_bag_dir} already exists. Removing old data...")
        subprocess.run(['rm', '-rf', str(output_bag_dir)])

    # 2. Get bag duration for progress bar
    print(f" Analysig input bag: {bag_name}...")
    info_proc = subprocess.run(['ros2', 'bag', 'info', bag_path], capture_output=True, text=True)
    duration_match = re.search(r"Duration:\s+(\d+\.\d+)s", info_proc.stdout)
    if not duration_match:
        print("Could not determine bag duration. Check if the bag path is correct.")
        return
    total_duration = float(duration_match.group(1))

    # 3. Start FAST-LIO (Headless)
    print(f"Launching FAST-LIO...")
    mapping_cmd = ['ros2', 'launch', 'fast_lio', 'mapping.launch.py', 
                   f'config_file:={config}', 'use_sim_time:=true', 'rviz:=false']
    mapping_proc = subprocess.Popen(mapping_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    # Give FAST-LIO a moment to initialize its publishers
    time.sleep(3)

    # 4. Start Recording Results
    print(f"Recording topics to: {output_bag_dir}")
    # Recording Odometry for APE, Cloud for map, and Path for visual summary
    record_cmd = ['ros2', 'bag', 'record', '/Odometry', '/cloud_registered', '/path', 
                  '-o', str(output_bag_dir)]
    record_proc = subprocess.Popen(record_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    # 5. Play the Raw Data Bag
    print(f"Playing input bag...")
    play_cmd = ['ros2', 'bag', 'play', bag_path, '--clock']
    # Start playback in background so we can track progress
    play_proc = subprocess.Popen(play_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    # 6. Progress Bar Monitoring
    start_time = time.time()
    print(f"Recording and Processing in progress...")
    
    try:
        while play_proc.poll() is None:  # While the bag is still playing
            elapsed = time.time() - start_time
            percent = min(100, (elapsed / total_duration) * 100)
            bar_length = 50
            filled_length = int(bar_length * percent // 100)
            bar = "█" * filled_length + "-" * (bar_length - filled_length)
            
            sys.stdout.write(f"\r|{bar}| {percent:.1f}% Complete")
            sys.stdout.flush()
            time.sleep(0.5)
            
        # Final 100% update
        sys.stdout.write(f"\r|{'█' * 50}| 100.0% Complete\n")
        sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\nStopping script...")
    finally:
        # 7. Cleanup and Shutdown
        print(f"Playback finished. Wrapping up recording...")
        
        # Kill record_proc first to ensure the bag is closed properly
        os.kill(record_proc.pid, signal.SIGINT)
        # Kill mapping
        os.kill(mapping_proc.pid, signal.SIGINT)
        # Kill playback if it's still running for some reason
        if play_proc.poll() is None:
            os.kill(play_proc.pid, signal.SIGINT)
        
        time.sleep(2)
        print(f"Done Results stored in: {output_bag_dir}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 record_fastlio.py [BAG_PATH] [CONFIG]")
        sys.exit(1)
    
    bag = sys.argv[1]
    cfg = sys.argv[2] if len(sys.argv) > 2 else "avia.yaml"
    run_and_record(bag, cfg)
