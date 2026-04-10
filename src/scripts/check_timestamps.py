#!/usr/bin/env python3
import sys
import argparse
import numpy as np
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def get_rosbag_options(path, storage_id='sqlite3'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    return storage_options, converter_options

def check_bag_timestamps(bag_path, topic_name):
    print(f"Analyzing hardware timestamps for '{topic_name}' in '{bag_path}'...")
    
    bag_path_obj = Path(bag_path)
    if bag_path_obj.is_file():
        read_path = str(bag_path_obj.parent)
    else:
        read_path = str(bag_path_obj)
        
    try:
        storage_opts, converter_opts = get_rosbag_options(read_path)
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_opts, converter_opts)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    if topic_name not in type_map:
        # Try to match with or without leading slash
        alt_topic_name = topic_name[1:] if topic_name.startswith('/') else '/' + topic_name
        if alt_topic_name in type_map:
            topic_name = alt_topic_name
        else:
            print(f"Error: Topic '{topic_name}' not found.")
            print(f"Available topics: {list(type_map.keys())}")
            return

    msg_type_str = type_map[topic_name]
    msg_class = get_message(msg_type_str)

    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)
    
    timestamps = []
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg = deserialize_message(data, msg_class)
        
        # Extract the exact hardware timestamp embedded in the message header
        try:
            t_sec = msg.header.stamp.sec
            t_nanosec = msg.header.stamp.nanosec
            t_total = t_sec + t_nanosec * 1e-9
            timestamps.append(t_total)
        except AttributeError:
            print("Error: The message on this topic does not have a standard header.stamp field.")
            return
                
    if len(timestamps) < 2:
        print("Not enough messages found on this topic to calculate deltas.")
        return
        
    # Calculate time differences (deltas) between consecutive messages
    deltas = np.diff(timestamps) * 1000.0 # Convert to milliseconds
    
    print("\n--- Timestamp Mathematical Analysis ---")
    print(f"Total Messages: {len(timestamps)}")
    print(f"Average Delta:  {np.mean(deltas):.4f} ms ({(1000.0/np.mean(deltas)):.2f} Hz)")
    print(f"Min Delta:      {np.min(deltas):.4f} ms")
    print(f"Max Delta:      {np.max(deltas):.4f} ms")
    print(f"Std Deviation:  {np.std(deltas):.4f} ms")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Verify if a bag's timestamps are pristine.")
    parser.add_argument("bag_path", help="Path to the ROS 2 bag")
    parser.add_argument("topic", help="Topic to analyze (e.g. /imu_raw)")
    args = parser.parse_args()
    check_bag_timestamps(args.bag_path, args.topic)