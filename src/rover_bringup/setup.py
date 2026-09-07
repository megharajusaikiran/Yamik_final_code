from setuptools import setup
from glob import glob
import os

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ultrasonic_two_pi5_node = rover_bringup.ultrasonic_two_pi5_node:main',
            'motor_node = rover_bringup.motor_node:main',
            'ultrasonic_node = rover_bringup.ultrasonic_node:main',
            'web_node = rover_bringup.web_node:main',
        ],
    },
)
