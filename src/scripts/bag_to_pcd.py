#!/usr/bin/env python3
import argparse
import sys
import os
import numpy as np
import psutil
import open3d as o3d
import concurrent.futures
import tempfile
import shutil
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

def get_rosbag_options(path, storage_id='sqlite3', serialization_format='cdr'):
    storage_options = StorageOptions(uri=path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def process_chunk_task(chunk_arrays, voxel_size, save_path):
    """Worker function to process and save a chunk to disk."""
    if not chunk_arrays:
        return 0
    
    # Concatenate numpy arrays
    all_pts = np.concatenate(chunk_arrays, axis=0)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_pts)
    
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        
    o3d.io.write_point_cloud(save_path, pcd, write_ascii=False, compressed=False, print_progress=False)
    return len(pcd.points)

def merge_pcd(bag_path, topic_name, output_path, voxel_size=0.1, storage_id='sqlite3', chunk_size=5000):
    """
    Reads a ROS2 bag, extracts PointCloud2 messages, merges them, and saves as PCD.
    """
    reader = SequentialReader()
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id=storage_id)
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    print(f"Processing bag: {bag_path}")
    print(f"Listening to topic: {topic_name}...")

    # Get total message count for progress bar
    total_msgs = 0
    try:
        metadata = rosbag2_py.Info().read_metadata(bag_path, storage_id)
        for topic_info in metadata.topics_with_message_count:
            if topic_info.topic_metadata.name == topic_name:
                total_msgs = topic_info.message_count
                break
    except Exception:
        pass # Fail silently if metadata can't be read, fallback to old print

    # Setup temporary directory for chunks
    temp_dir = tempfile.mkdtemp()
    print(f"Using temporary directory for chunks: {temp_dir}")

    executor = concurrent.futures.ProcessPoolExecutor()
    futures = []
    chunk_files = []
    chunk_points = []
    msg_count = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if topic == topic_name:
            msg = deserialize_message(data, PointCloud2)
            
            # Extract (x, y, z) points
            # specific field names ensure we don't grab intensity/rgb by accident if not needed
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            data_list = list(gen)
            
            if data_list:
                try:
                    points = np.array(data_list, dtype=np.float32)
                except TypeError:
                    # Handle structured numpy scalars (void types)
                    raw = np.array(data_list)
                    points = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float32)

                if points.ndim == 1:
                    points = points.reshape(-1, 3)
                
                chunk_points.append(points)
            
            msg_count += 1
            
            # Check chunk size or RAM safety
            mem_usage = psutil.virtual_memory().percent
            if len(chunk_points) >= chunk_size or (len(chunk_points) > 0 and mem_usage > 80.0):
                # Submit chunk to worker
                chunk_filename = os.path.join(temp_dir, f"chunk_{len(futures)}.pcd")
                chunk_files.append(chunk_filename)
                
                # Submit task (copy list to avoid reference issues)
                f = executor.submit(process_chunk_task, chunk_points, voxel_size, chunk_filename)
                futures.append(f)
                
                chunk_points = [] # Clear buffer
                
                # If RAM is critical, wait for workers to clear backlog
                if mem_usage > 85.0:
                    concurrent.futures.wait(futures, timeout=None)

            if total_msgs > 0:
                percent = (msg_count / total_msgs) * 100
                sys.stdout.write(f"\rProcessing: {percent:.1f}% | Chunks: {len(futures)} | RAM: {mem_usage:.1f}%")
                sys.stdout.flush()

    # Process remaining points
    if chunk_points:
        chunk_filename = os.path.join(temp_dir, f"chunk_{len(futures)}.pcd")
        chunk_files.append(chunk_filename)
        f = executor.submit(process_chunk_task, chunk_points, voxel_size, chunk_filename)
        futures.append(f)

    print("\nWaiting for background tasks to finish...")
    concurrent.futures.wait(futures)
    executor.shutdown()

    # Calculate total points
    total_points = 0
    try:
        for f in futures:
            total_points += f.result()
    except Exception as e:
        print(f"Error in worker process: {e}")
        shutil.rmtree(temp_dir)
        return

    if total_points == 0:
        print("\nNo point cloud data found!")
        shutil.rmtree(temp_dir)
        return

    # Glue chunks together
    print(f"Gluing {len(chunk_files)} chunks into {output_path}...")
    
    try:
        with open(output_path, 'wb') as f_out:
            # Write PCD Header
            header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {total_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {total_points}
DATA binary
"""
            f_out.write(header.encode('ascii'))
            
            # Append Data
            for i, pcd_path in enumerate(chunk_files):
                # Load chunk to get binary data
                chunk_pcd = o3d.io.read_point_cloud(pcd_path)
                pts = np.asarray(chunk_pcd.points, dtype=np.float32)
                f_out.write(pts.tobytes())
                
                if i % 10 == 0:
                    sys.stdout.write(f"\rMerging chunk {i+1}/{len(chunk_files)}")
                    sys.stdout.flush()
        
        print(f"\nDone! Saved {total_points} points.")
        
    finally:
        # Cleanup temp files
        shutil.rmtree(temp_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS2 Bag to Merged PCD")
    parser.add_argument("bag_path", help="Path to the input ROS2 bag (folder or .mcap)")
    parser.add_argument("--topic", default="/cloud_registered", help="Topic name (default: /cloud_registered)")
    parser.add_argument("--output", default="final_map.pcd", help="Output filename")
    parser.add_argument("--voxel", type=float, default=0.1, help="Voxel downsample size in meters (0 to disable)")
    parser.add_argument("--storage_id", default="sqlite3", help="Storage format (sqlite3 or mcap)")
    parser.add_argument("--chunk_size", type=int, default=1000, help="Target frames to process before merging (default: 1000)")

    args = parser.parse_args()
    merge_pcd(args.bag_path, args.topic, args.output, args.voxel, args.storage_id, args.chunk_size)
