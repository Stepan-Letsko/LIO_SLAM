#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import signal
import datetime
import os

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder_node')
        self.subscription = self.create_subscription(
            Bool, 
            '/foxglove/record_bag', 
            self.listener_callback, 
            10
        )
        self.process = None
        # Ensure the bags directory exists inside the container
        self.bag_dir = "/root/ros2_ws/bags"
        os.makedirs(self.bag_dir, exist_ok=True)
        
        # Publisher to broadcast current recording status
        self.status_pub = self.create_publisher(Bool, '/foxglove/recording_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status) # Update UI every 1 sec
        
        self.get_logger().info(f"Bag Recorder Ready. Waiting for signal on /foxglove/record_bag...")

    def publish_status(self):
        msg = Bool()
        if self.process is not None and self.process.poll() is not None:
            self.process = None # Process died or finished
        msg.data = (self.process is not None)
        self.status_pub.publish(msg)

    def listener_callback(self, msg):
        # Start Recording
        if msg.data and self.process is None:
            timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
            bag_name = f"foxglove_rec_{timestamp}"
            full_path = os.path.join(self.bag_dir, bag_name)
            
            # Topics to record (SLAM output + Raw Odometry)
            topics = ["/cloud_registered", "/Odometry", "/path", "/tf", "/tf_static"]
            
            cmd = ["ros2", "bag", "record", "-o", full_path] + topics
            
            self.get_logger().warn(f"STARTING RECORDING: {bag_name}")
            # Use Popen to run in background
            self.process = subprocess.Popen(cmd)
            self.publish_status() # Force immediate UI update

        # Stop Recording
        elif not msg.data and self.process is not None:
            self.get_logger().warn("STOPPING RECORDING...")
            self.process.send_signal(signal.SIGINT) # Graceful shutdown to flush buffer
            self.process.wait()
            self.process = None
            self.get_logger().info("Recording stopped and saved.")
            self.publish_status() # Force immediate UI update

def main(args=None):
    rclpy.init(args=args)
    node = BagRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()