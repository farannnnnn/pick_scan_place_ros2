#!/usr/bin/env python3

# qr_decision_node.py
# Author: Syed Faran Ali
# Course: MAI605 - Robotic Systems (ROS2)
# Description: Simulates QR code scanning and bin decision logic.
#              In a real system this would use zbar_ros to decode
#              actual QR codes from a camera. Simulated here due
#              to WSL2 camera driver limitations.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random


class QRDecisionNode(Node):

    def __init__(self):
        super().__init__('qr_decision_node')

        # Subscribe to scan trigger from motion_node
        # motion_node publishes SCAN_NOW when robot reaches scan pose
        self.scan_sub = self.create_subscription(
            String, '/scan_trigger', self.scan_callback, 10)

        # Publish decoded QR result back to motion_node
        # motion_node uses this to decide which bin to place object in
        self.result_pub = self.create_publisher(String, '/qr_result', 10)

        # Publish human readable decision log for monitoring
        self.decision_pub = self.create_publisher(String, '/decision_log', 10)

        self.get_logger().info('QR Decision Node initialized. Waiting for scan trigger...')

    def scan_callback(self, msg):
        # Only respond to SCAN_NOW trigger, ignore anything else
        if msg.data != 'SCAN_NOW':
            return

        self.get_logger().info('Scan trigger received! Simulating QR decode...')

        # Simulate QR code decoding
        # In a real system: subscribe to /barcode topic from zbar_ros
        # zbar_ros reads from camera and publishes decoded string
        # Here we randomly return A or B to simulate two different QR codes
        qr_value = random.choice(['A', 'B'])
        self.get_logger().info(f'QR code decoded: {qr_value}')

        # Send the decoded value back to motion_node
        result_msg = String()
        result_msg.data = qr_value
        self.result_pub.publish(result_msg)

        # Build a human readable decision message and publish it
        # A = left bin, B = right bin
        if qr_value == 'A':
            decision = f'QR={qr_value} → Routing object to BIN A (left side)'
        else:
            decision = f'QR={qr_value} → Routing object to BIN B (right side)'

        log_msg = String()
        log_msg.data = decision
        self.decision_pub.publish(log_msg)
        self.get_logger().info(f'Decision: {decision}')


def main(args=None):
    rclpy.init(args=args)
    node = QRDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
