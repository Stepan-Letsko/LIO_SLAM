import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # =========================================
    # 1. Hardware Drivers
    # =========================================

    # SBG IMU Driver (Configured for 200Hz ENU orientation)
    sbg_pkg = get_package_share_directory('sbg_driver')
    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sbg_pkg, 'launch', 'sbg_device_launch.py')
        )
    )

    # Hesai LiDAR Driver (Configured for 10Hz, Strongest Return)
    hesai_pkg = get_package_share_directory('hesai_ros_driver')
    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hesai_pkg, 'launch', 'start.py')
        )
    )

    # =========================================
    # 2. SLAM Algorithm
    # =========================================

    # FAST-LIO 2
    # - Uses 'hesai.yaml' config
    # - Disables local Rviz (saves GPU/CPU on the Pi)
    fast_lio_pkg = get_package_share_directory('fast_lio')
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'hesai.yaml',
            'rviz': 'false'
        }.items()
    )

    # =========================================
    # 3. Visualization Bridge
    # =========================================

    # Foxglove Bridge
    # Creates a WebSocket at ws://<PI_IP>:8765
    foxglove_launch = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765, 
            'address': '0.0.0.0', 
            'send_buffer_limit': 10000000,
            'topic_whitelist': ['^(?!/lidar_points$).*']
        }]
    )

    # Bag Recorder Listener
    # Runs the python script to listen for Foxglove button presses
    bag_recorder_node = ExecuteProcess(
        cmd=['python3', '/root/ros2_ws/src/FAST_LIO_ROS2/src/bag_trigger.py'],
        output='screen'
    )

    # Delay SLAM slightly to ensure drivers are fully up and publishing
    delayed_fast_lio = TimerAction(period=3.0, actions=[fast_lio_launch])

    return LaunchDescription([
        sbg_launch,
        hesai_launch,
        foxglove_launch,
        bag_recorder_node,
        delayed_fast_lio
    ])