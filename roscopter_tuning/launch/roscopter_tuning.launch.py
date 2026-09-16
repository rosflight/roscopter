from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Create the package directory
    roscopter_share = FindPackageShare('roscopter')

    autopilot_params = PathJoinSubstitution([roscopter_share, 'params', 'multirotor.yaml'])

    return LaunchDescription([
        Node(
            package='roscopter',
            executable='controller',
            name='autopilot',
            parameters = [autopilot_params,
                          {'pitch_tuning_override': False},
                          {'roll_tuning_override': True}],
            output = 'screen',
            remappings=[('estimated_state', 'state')]
        ),
        Node(
            package='roscopter',
            executable='ekf_node',
            name='estimator'
        ),
        Node(
            package='roscopter_tuning',
            executable='signal_generator',
            name='signal_generator',
            output = 'screen'
        ),
        Node(
            package='roscopter',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[autopilot_params],
            remappings=[('estimated_state', 'state')]
        )
    ])
