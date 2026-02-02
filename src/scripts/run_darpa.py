#!/usr/bin/env python3
import subprocess
import sys
import time
import signal
import os
from pathlib import Path

def run_darpa_analysis(bag_path):
    bag_path = Path(bag_path)
    if not bag_path.exists():
        print(f"Error: Bag file {bag_path} not found.")
        return

    print(f"========================================")
    print(f" DARPA DATASET ANALYSIS")
    print(f" Bag: {bag_path.name}")
    print(f"========================================")

    # 1. Launch Decoder
    print("\n[1/3] Launching Velodyne Decoder...")
    # We use the launch file we just created in FAST_LIO_ROS2
    decoder_cmd = ['ros2', 'launch', 'fast_lio', 'decoder.launch.py']
    proc_decoder = subprocess.Popen(decoder_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # 2. Launch FAST-LIO
    print("[2/3] Launching FAST-LIO (dapra.yaml)...")
    # We pipe stdout so we can see the logs (like Novelty detection)
    fastlio_cmd = ['ros2', 'launch', 'fast_lio', 'mapping.launch.py', 'config_file:=dapra.yaml']
    proc_fastlio = subprocess.Popen(fastlio_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    # Wait for initialization
    print("      Waiting 5 seconds for nodes to initialize...")
    time.sleep(5)

    # 3. Play Bag
    print(f"[3/3] Playing Bag: {bag_path.name}...")
    # --clock is crucial for sim time
    bag_cmd = ['ros2', 'bag', 'play', str(bag_path), '--clock']
    proc_bag = subprocess.Popen(bag_cmd)

    print("\n--- SYSTEM RUNNING (Press Ctrl+C to Stop) ---")
    
    try:
        # Loop while bag is playing
        while proc_bag.poll() is None:
            # Read FAST-LIO output line by line and print it
            line = proc_fastlio.stdout.readline()
            if line:
                print(line.strip())
            
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        print("\nShutting down...")
        # Kill processes in reverse order
        if proc_bag.poll() is None:
            proc_bag.send_signal(signal.SIGINT)
        
        if proc_fastlio.poll() is None:
            proc_fastlio.send_signal(signal.SIGINT)
            
        if proc_decoder.poll() is None:
            proc_decoder.send_signal(signal.SIGINT)
            
        # Wait for them to exit
        proc_bag.wait()
        proc_fastlio.wait()
        proc_decoder.wait()
        print("Done.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 run_darpa.py <path_to_bag>")
    else:
        run_darpa_analysis(sys.argv[1])