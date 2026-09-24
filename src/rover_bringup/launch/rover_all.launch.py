from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_bringup',
            executable='rover_sensors_gps_node',
            name='rover_sensors_gps_node',
            output='screen'
        ),

        Node(
            package='rover_bringup',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'port': 9090}
            ]
        ),
    ])
