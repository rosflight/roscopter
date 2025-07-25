import os 
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    roscopter_gcs_dir = get_package_share_directory('roscopter_gcs')

    rviz2_config_file = roscopter_gcs_dir + '/config/roscopter_gcs.rviz'
    rviz2_splash_file = roscopter_gcs_dir + '/resource/logo.png'

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x','0','--y','0','--z','0','--yaw','0','--pitch','0','--roll','3.1415926535','--frame-id','world','--child-frame-id','NED']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x','0','--y','0','--z','0','--yaw','-1.570796326','--pitch','0','--roll','3.1415926535','--frame-id','aircraft_body','--child-frame-id','stl_frame']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d',rviz2_config_file, '-s', rviz2_splash_file]
        ),
        Node(
            package='roscopter_gcs',
            executable='rviz_waypoint_publisher',
            remappings=[('estimated_state', 'state')]
        )
    ])