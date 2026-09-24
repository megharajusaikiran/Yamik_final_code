from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    lidar = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pi/rover_ws/src/ydlidar_x2-py-ros2-node/ydlidarpy/ydlidarpy/x2_node.py'
        ],
        name='ydlidar_x2',
        output='screen'
    )

    rosbridge = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'rosbridge_server',
            'rosbridge_websocket_launch.xml'
        ],
        name='rosbridge',
        output='screen'
    )

    safety = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'rover_bringup',
            'lidar_safety_node'
        ],
        name='lidar_safety_node',
        output='screen'
    )

    return LaunchDescription([
        lidar,
        rosbridge,
        safety
    ])
