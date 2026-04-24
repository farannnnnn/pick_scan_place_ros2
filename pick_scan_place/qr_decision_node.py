#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class QRDecisionNode(Node):
    def __init__(self):
        super().__init__('qr_decision_node')
        self.scan_sub = self.create_subscription(String, '/scan_trigger', self.scan_callback, 10)
        self.result_pub = self.create_publisher(String, '/qr_result', 10)
        self.decision_pub = self.create_publisher(String, '/decision_log', 10)
        self.get_logger().info('QR Decision Node initialized. Waiting for scan trigger...')

    def scan_callback(self, msg):
        if msg.data != 'SCAN_NOW':
            return
        self.get_logger().info('Scan trigger received! Simulating QR decode...')
        qr_value = random.choice(['A', 'B'])
        self.get_logger().info(f'QR code decoded: {qr_value}')
        result_msg = String()
        result_msg.data = qr_value
        self.result_pub.publish(result_msg)
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
