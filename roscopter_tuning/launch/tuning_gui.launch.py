from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    roscopter_tuning_share = FindPackageShare('roscopter_tuning')

    tuning_config = PathJoinSubstitution([
        roscopter_tuning_share,
        'resources',
        'param_tuning_config.yaml'
    ])

    return LaunchDescription([
        Node(
            package='rosflight_rqt_plugins',
            executable='param_tuning',
            name='tuning_gui',
            output='screen',
            arguments=['--config-filepath', tuning_config]
        )
    ])

