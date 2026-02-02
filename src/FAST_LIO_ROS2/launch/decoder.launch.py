from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Locate the standard VLP16 calibration file provided by the velodyne package
    calibration_file = os.path.join(
        get_package_share_directory('velodyne_pointcloud'),
        'params',
        'VLP16db.yaml'
    )

    return LaunchDescription([
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[{
                'model': 'VLP16',
                'calibration': calibration_file,
                'min_range': 0.4,
                'max_range': 130.0,
                'view_direction': 0.0,
                'view_width': 6.28,
            }],
            # Remap the input from the bag (/lidar/packets) to what the node expects
            remappings=[('/velodyne_packets', '/lidar/packets')]
        )
    ])