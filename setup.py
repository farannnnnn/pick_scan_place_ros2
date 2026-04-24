from setuptools import setup
import os
from glob import glob

package_name = 'pick_scan_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Register the package with ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farhan',
    maintainer_email='farhan@todo.todo',
    description='Pick-Scan-Place pipeline using ROS 2 and MoveIt 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registers motion_node as a runnable executable
            'motion_node = pick_scan_place.motion_node:main',
            # Registers qr_decision_node as a runnable executable
            'qr_decision_node = pick_scan_place.qr_decision_node:main',
        ],
    },
)
