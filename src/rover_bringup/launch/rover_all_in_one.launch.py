from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():
    # Your original rover: motor + ultrasonic + (maybe rosbridge)
    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(__file__).parent / 'rover_all.launch.py'
        )
    )

    # LiDAR node
    lidar = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pi/rover_ws/src/ydlidar_x2-py-ros2-node/ydlidarpy/ydlidarpy/x2_node.py'
        ],
        name='ydlidar_x2',
        output='screen'
    )

    # LiDAR safety
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
        rover,
        lidar,
        safety
    ])
