from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():
    roscopter_dir = get_package_share_directory('roscopter')

    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                roscopter_dir,
                'launch',
                'roscopter.launch.py'
            )
        )
    )

    return LaunchDescription([
        base_launch_include,
        Node(
            package='roscopter_sim',
            executable='sim_state_transcriber',
            output='screen',
            name='roscopter_truth'
        )
    ])
