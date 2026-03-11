#!/bin/bash
set -e

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Define the path to the driver
DRIVER_DIR="/root/ros2_ws/src/livox_ros_driver"

# Check if the driver exists and if it still has the ROS 2 package definition named 'package_ROS2.xml'
if [ -d "$DRIVER_DIR" ] && [ -f "$DRIVER_DIR/package_ROS2.xml" ]; then
    echo "Auto-configuring Livox Driver for ROS 2..."
    # Rename the ROS 2 package file to be the active one
    mv "$DRIVER_DIR/package_ROS2.xml" "$DRIVER_DIR/package.xml"
    # Remove the ROS 1 package file to avoid confusion
    rm -f "$DRIVER_DIR/package_ROS1.xml"
fi

# Source the workspace local setup if available (for ROS 2, it's install/setup.bash)
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source "/root/ros2_ws/install/setup.bash"
fi

# Execute the command passed to the container (default is bash)
exec "$@"