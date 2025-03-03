import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('raisin_bridge'),
        'config',
        'params.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='raisin_bridge',
            executable='raisin_bridge_node',
            parameters=[config]
        )
    ])

