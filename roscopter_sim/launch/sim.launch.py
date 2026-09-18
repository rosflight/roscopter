from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    roscopter_share = FindPackageShare('roscopter')

    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([roscopter_share, 'launch', 'roscopter.launch.py'])
        )
    )

    return LaunchDescription(
        [
            base_launch_include,
            Node(
                package='roscopter_gcs',
                executable='rviz_waypoint_publisher',
                output='screen',
            ),
            Node(
                package='roscopter_sim',
                executable='sim_state_transcriber',
                output='screen',
                name='roscopter_truth',
            ),
        ]
    )
