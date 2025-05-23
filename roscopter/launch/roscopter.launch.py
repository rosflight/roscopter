import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roscopter_dir = get_package_share_directory('roscopter')
    param_file = os.path.join(roscopter_dir, 'params', 'multirotor.yaml')

    state_remap_arg = DeclareLaunchArgument(
        "state_topic",
        default_value="estimated_state",
        description="Topic name the every node but the estimator will listen to for state information."
    )
    state_remap = LaunchConfiguration('state_topic')

    return LaunchDescription([
        state_remap_arg,
        Node(
            package='roscopter',
            executable='controller',
            name='autopilot',
            output='screen',
            parameters=[param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='estimator',
            name='estimator',
            output='screen'
        ),
        Node(
            package='roscopter',
            executable='path_manager',
            name='path_manager',
            output='screen',
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='path_planner',
            name='path_planner',
            output='screen',
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='ext_att_transcriber',
            name='external_attitude_transcriber',
            output='screen',
            remappings=[('estimated_state', state_remap)]
        ),
    ])
