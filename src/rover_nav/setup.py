from setuptools import setup

package_name = 'rover_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover_nav.launch.py', 'launch/rover_master.launch.py']),    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@example.com',
    description='GPS + IMU + waypoint navigation for YAMIK rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_node = rover_nav.gps_node:main',
            'imu_node = rover_nav.imu_node:main',
            'nav_node = rover_nav.nav_node:main',
        ],
    },
)
