from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_nav',
            executable='gps_node',
            name='gps_node',
            output='screen',
        ),
        Node(
            package='rover_nav',
            executable='imu_node',
            name='bno055_node',
            output='screen',
        ),
        Node(
            package='rover_nav',
            executable='nav_node',
            name='nav_node',
            output='screen',
        ),
    ])
