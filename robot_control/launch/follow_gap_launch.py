import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    base_path = os.path.realpath(get_package_share_directory('robot_control'))
    rviz_path=base_path+'/config/follow_gap_config.rviz'
    
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='follow_gap',
            name='reactive_controler'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', str(rviz_path)]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mimic',
            arguments = ["0", "0", "0", "0", "0", "0", "chassis", "laser_link"]
        )
    ])
