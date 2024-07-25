import os
import sys
from launch import LaunchDescription
from launch.descriptions import executable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    roscopter_dir = get_package_share_directory('roscopter')

    # Determine the appropriate control scheme.
    control_type = "default"

    for arg in sys.argv:
        if arg.startswith("control_type:="):
            control_type = arg.split(":=")[1]

    autopilot_params = os.path.join(
        roscopter_dir,
        'params',
        'multirotor.yaml'
    )

    return LaunchDescription([
        Node(
            package='roscopter',
            executable='controller',
            name='autopilot',
            parameters = [autopilot_params,
                          {'pitch_tuning_override': False},
                          {'roll_tuning_override': True}],
            output = 'screen',
            arguments = [control_type],
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
        )
    ])
