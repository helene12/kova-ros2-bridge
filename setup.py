from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'kova_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kova Systems',
    maintainer_email='dev@kovasystems.com',
    description='ROS2 integration bridge for the Kova ecosystem',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kova_bridge_node = kova_ros2_bridge.kova_bridge_node:main',
            'sensor_processor_node = kova_ros2_bridge.sensor_processor_node:main',
            'reward_claimer_node = kova_ros2_bridge.reward_claimer_node:main',
        ],
    },
)
