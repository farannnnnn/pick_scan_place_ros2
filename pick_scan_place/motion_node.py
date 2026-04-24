#!/usr/bin/env python3
"""
motion_node.py - Pick-Scan-Place pipeline controller for Panda robot.
Controls robot movement through pick, scan, and place phases.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        # Publishers
        self.status_pub = self.create_publisher(String, '/pipeline_status', 10)
        self.scan_trigger_pub = self.create_publisher(String, '/scan_trigger', 10)
        # Subscriber for QR result
        self.qr_sub = self.create_subscription(String, '/qr_result', self.qr_callback, 10)
        # State variables
        self.qr_result = None
        self.step = 0  # Pipeline step tracker
        self.waiting_for_qr = False
        # Timer drives the pipeline (runs every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.pipeline_step)
        self.step_timer = 0
        self.get_logger().info('Motion Node initialized. Starting pipeline...')

    def qr_callback(self, msg):
        """Receives QR decode result from qr_decision_node."""
        self.qr_result = msg.data
        self.get_logger().info(f'QR result received: {self.qr_result}')
        self.waiting_for_qr = False

    def publish_status(self, status):
        """Publishes and logs pipeline status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Pipeline status: {status}')

    def pipeline_step(self):
        """Non-blocking pipeline using a step machine."""
        self.step_timer += 0.1

        # Step 0: Start
        if self.step == 0:
            self.get_logger().info('=== Starting Pick-Scan-Place Pipeline ===')
            self.step = 1
            self.step_timer = 0

        # Step 1: Pick phase - wait 2 seconds
        elif self.step == 1:
            self.publish_status('PICKING')
            self.get_logger().info('Moving to pick position: (0.4, 0.0, 0.4)')
            self.step = 2
            self.step_timer = 0

        # Step 2: Wait at pick position
        elif self.step == 2 and self.step_timer >= 2.0:
            self.step = 3
            self.step_timer = 0

        # Step 3: Scan phase - send trigger
        elif self.step == 3:
            self.publish_status('SCANNING')
            self.get_logger().info('Moving to scan position: (0.3, 0.3, 0.6)')
            trigger_msg = String()
            trigger_msg.data = 'SCAN_NOW'
            self.scan_trigger_pub.publish(trigger_msg)
            self.get_logger().info('Scan trigger sent. Waiting for QR result...')
            self.waiting_for_qr = True
            self.step = 4
            self.step_timer = 0

        # Step 4: Wait for QR result (up to 5 seconds)
        elif self.step == 4:
            if not self.waiting_for_qr or self.step_timer >= 5.0:
                self.step = 5
                self.step_timer = 0

        # Step 5: Place phase based on QR result
        elif self.step == 5:
            if self.qr_result == 'A':
                self.publish_status('PLACING_A')
                self.get_logger().info('QR=A → Placing in Bin A (left side)')
            else:
                self.publish_status('PLACING_B')
                self.get_logger().info('QR=B → Placing in Bin B (right side)')
            self.step = 6
            self.step_timer = 0

        # Step 6: Wait at place position
        elif self.step == 6 and self.step_timer >= 2.0:
            self.step = 7

        # Step 7: Complete
        elif self.step == 7:
            self.publish_status('COMPLETE')
            self.get_logger().info('=== Pipeline Complete ===')
            self.timer.cancel()  # Stop the timer

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
