import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roscopter_dir = get_package_share_directory('roscopter')

    return LaunchDescription([
        Node(
            package='roscopter',
            executable='controller',
            name='autopilot'
        ),
        Node(
            package='roscopter',
            executable='ekf_node',
            name='estimator'
        )
    ])