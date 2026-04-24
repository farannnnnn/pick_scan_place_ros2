#!/usr/bin/env python3
"""
demo.launch.py
--------------
Main launch file for the pick-scan-place pipeline.
Launches three things together:
1. MoveIt 2 with Panda robot in RViz
2. motion_node - controls robot movement
3. qr_decision_node - handles QR scanning logic
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── Get package paths ────────────────────────────────────────────
    moveit_tutorials_dir = get_package_share_directory('moveit2_tutorials')
    pick_scan_place_dir = get_package_share_directory('pick_scan_place')

    # ── MoveIt 2 Panda Demo ──────────────────────────────────────────
    # Launches the Panda robot with MoveIt 2 and RViz visualization
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_tutorials_dir,
                'launch',
                'demo.launch.py'
            )
        )
    )

    # ── Motion Node ──────────────────────────────────────────────────
    # Controls the pick-scan-place movement pipeline
    motion_node = Node(
        package='pick_scan_place',
        executable='motion_node',
        name='motion_node',
        output='screen',
        parameters=[
            os.path.join(
                pick_scan_place_dir,
                'config',
                'poses.yaml'
            )
        ]
    )

    # ── QR Decision Node ─────────────────────────────────────────────
    # Handles QR code simulation and bin decision logic
    qr_decision_node = Node(
        package='pick_scan_place',
        executable='qr_decision_node',
        name='qr_decision_node',
        output='screen'
    )

    return LaunchDescription([
        moveit_demo,
        motion_node,
        qr_decision_node,
    ])
