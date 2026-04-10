#!/usr/bin/env python3
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
