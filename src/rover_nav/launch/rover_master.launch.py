from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rover_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_bringup'),
                'launch',
                'rover_all_in_one.launch.py'
            )
        )
    )

    gps = Node(
        package='rover_nav',
        executable='gps_node',
        name='gps_node',
        output='screen',
    )

    imu = Node(
        package='rover_nav',
        executable='imu_node',
        name='bno055_node',
        output='screen',
    )

    nav = Node(
        package='rover_nav',
        executable='nav_node',
        name='nav_node',
        output='screen',
    )

    return LaunchDescription([rover_stack, gps, imu, nav])
