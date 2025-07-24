import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roscopter_dir = get_package_share_directory('roscopter')
    controller_param_file = os.path.join(roscopter_dir, 'params', 'multirotor.yaml')
    estimator_param_file = os.path.join(roscopter_dir, 'params', 'estimator.yaml')
    
    hotstart_estimator_arg = DeclareLaunchArgument(
        "hotstart_estimator",
        default_value="false",
        description="Whether the estimator will hotstart based on the contents of 'params/hotstart'"
    )
    hotstart_estimator = LaunchConfiguration('hotstart_estimator')

    state_remap_arg = DeclareLaunchArgument(
        "state_topic",
        default_value="estimated_state",
        description="Topic name the every node but the estimator will listen to for state information."
    )
    state_remap = LaunchConfiguration('state_topic')

    return LaunchDescription([
        state_remap_arg,
        hotstart_estimator_arg,
        Node(
            package='roscopter',
            executable='controller',
            name='autopilot',
            output='screen',
            parameters=[controller_param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[controller_param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='estimator',
            name='estimator',
            output='screen',
            parameters=[estimator_param_file, {"hotstart_estimator": hotstart_estimator}],
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
