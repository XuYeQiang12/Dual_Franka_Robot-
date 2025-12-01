from setuptools import setup
import os
from glob import glob

package_name = 'dual_franka_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Dual Franka Robot MuJoCo Simulation with ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_franka_sim = dual_franka_ros2.mujoco_dual_franka_sim:main',
            'test_controller = dual_franka_ros2.test_controller:main',
        ],
    },
)
