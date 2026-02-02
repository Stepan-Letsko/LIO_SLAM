#!/usr/bin/env python3
import argparse
import sys
import os
import numpy as np
import open3d as o3d
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

def merge_pcd(bag_path, topic_name, output_path, voxel_size=0.1, storage_id='sqlite3'):
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

    merged_points = []
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
                merged_points.append(points)
            
            msg_count += 1
            
            if total_msgs > 0:
                percent = (msg_count / total_msgs) * 100
                bar_len = 40
                filled = int(bar_len * msg_count // total_msgs)
                bar = '█' * filled + '-' * (bar_len - filled)
                sys.stdout.write(f"\r|{bar}| {percent:.1f}%")
                sys.stdout.flush()
            elif msg_count % 100 == 0:
                print(f"Processed {msg_count} frames...")

    if not merged_points:
        print("\nNo point cloud data found!")
        return

    # Concatenate all frames into one massive array
    print("\nMerging point clouds...")
    all_points = np.concatenate(merged_points, axis=0)
    
    # Create Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)

    # Optional: Downsample to save disk space and remove duplicates
    if voxel_size > 0:
        print(f"Downsampling with voxel size {voxel_size}m...")
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Save to file
    print(f"Saving to {output_path}...")
    o3d.io.write_point_cloud(output_path, pcd)
    print(f"Done! Saved {len(pcd.points)} points.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS2 Bag to Merged PCD")
    parser.add_argument("bag_path", help="Path to the input ROS2 bag (folder or .mcap)")
    parser.add_argument("--topic", default="/cloud_registered", help="Topic name (default: /cloud_registered)")
    parser.add_argument("--output", default="final_map.pcd", help="Output filename")
    parser.add_argument("--voxel", type=float, default=0.1, help="Voxel downsample size in meters (0 to disable)")
    parser.add_argument("--storage_id", default="sqlite3", help="Storage format (sqlite3 or mcap)")

    args = parser.parse_args()
    merge_pcd(args.bag_path, args.topic, args.output, args.voxel, args.storage_id)