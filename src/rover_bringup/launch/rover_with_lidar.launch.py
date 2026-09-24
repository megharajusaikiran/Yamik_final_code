from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():
    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(__file__).parent.parent / 'rover_bringup' / 'launch' / 'rover_bringup.launch.py'
        )
    )

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

    delay_rest = RegisterEventHandler(
        OnProcessStart(
            target_action=rover_launch,
            on_start=[lidar, rosbridge, safety]
        )
    )

    return LaunchDescription([
        rover_launch,
        delay_rest
    ])
