#!/usr/bin/env python3
"""
motion_node.py
--------------
Pick-Scan-Place pipeline controller for Panda robot.
Uses ROS 2 action client to send goals to MoveIt 2 move_group,
causing the Panda arm to actually move in RViz.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, Constraints, JointConstraint
from sensor_msgs.msg import JointState
import time

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        # ── Action Client for MoveIt 2 ───────────────────────────────
        self.move_client = ActionClient(self, MoveGroup, '/move_action')

        # ── Publishers ───────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/pipeline_status', 10)
        self.scan_trigger_pub = self.create_publisher(String, '/scan_trigger', 10)

        # ── Subscriber ───────────────────────────────────────────────
        self.qr_sub = self.create_subscription(String, '/qr_result', self.qr_callback, 10)

        # ── State ────────────────────────────────────────────────────
        self.qr_result = None
        self.step = 0
        self.step_timer = 0.0
        self.waiting_for_qr = False
        self.motion_complete = False
        self.motion_success = False

        self.get_logger().info('Motion Node initialized. Waiting for MoveIt 2...')

        # Wait for move_group action server
        self.move_client.wait_for_server()
        self.get_logger().info('MoveIt 2 connected! Starting pipeline in 3 seconds...')

        # Timer drives pipeline every 0.1 seconds
        self.timer = self.create_timer(0.1, self.pipeline_step)
        self.step_timer = 0.0

    def qr_callback(self, msg):
        """Receives QR result from qr_decision_node."""
        self.qr_result = msg.data
        self.waiting_for_qr = False
        self.get_logger().info(f'QR result received: {self.qr_result}')

    def publish_status(self, status):
        """Publishes and logs pipeline status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Pipeline status: {status}')

    def move_to_joint_positions(self, joint_positions, description='target'):
        """
        Sends a joint position goal to MoveIt 2 move_group.
        joint_positions: list of 7 joint angles in radians for panda_arm
        """
        self.get_logger().info(f'Planning motion to {description}...')
        self.motion_complete = False
        self.motion_success = False

        # Build the motion plan request
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'panda_arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # Set workspace bounds
        goal.request.workspace_parameters = WorkspaceParameters()
        goal.request.workspace_parameters.header.frame_id = 'panda_link0'
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0

        # Joint names for panda_arm
        joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # Build joint constraints
        goal.request.goal_constraints.append(Constraints())
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.request.goal_constraints[0].joint_constraints.append(jc)

        # Send goal and register callbacks
        future = self.move_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        """Called during motion planning."""
        pass

    def goal_response_callback(self, future):
        """Called when move_group accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Motion goal rejected!')
            self.motion_complete = True
            self.motion_success = False
            return
        self.get_logger().info('Motion goal accepted! Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Called when motion execution completes."""
        result = future.result().result
        error_code = result.error_code.val
        if error_code == 1:
            self.get_logger().info('Motion executed successfully!')
            self.motion_success = True
        else:
            self.get_logger().warn(f'Motion failed with error code: {error_code}')
            self.motion_success = False
        self.motion_complete = True

    def pipeline_step(self):
        """Non-blocking pipeline state machine."""
        self.step_timer += 0.1

        # Step 0: Initial delay
        if self.step == 0 and self.step_timer >= 3.0:
            self.get_logger().info('=== Starting Pick-Scan-Place Pipeline ===')
            # Move to ready/home position
            # Panda ready pose joint angles
            self.move_to_joint_positions(
                [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
                'home position'
            )
            self.step = 1
            self.step_timer = 0.0

        # Step 1: Wait for home motion to complete
        elif self.step == 1 and self.motion_complete:
            self.step = 2
            self.step_timer = 0.0

        # Step 2: Pick phase
        elif self.step == 2 and self.step_timer >= 0.5:
            self.publish_status('PICKING')
            # Pick position joint angles
            self.move_to_joint_positions(
                [0.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785],
                'pick position'
            )
            self.step = 3
            self.step_timer = 0.0

        # Step 3: Wait for pick motion
        elif self.step == 3 and self.motion_complete:
            self.get_logger().info('At pick position! Grasping object...')
            self.step = 4
            self.step_timer = 0.0

        # Step 4: Lift phase
        elif self.step == 4 and self.step_timer >= 1.0:
            self.publish_status('LIFTING')
            self.move_to_joint_positions(
                [0.0, -0.2, 0.0, -2.0, 0.0, 1.8, 0.785],
                'lift position'
            )
            self.step = 5
            self.step_timer = 0.0

        # Step 5: Wait for lift motion
        elif self.step == 5 and self.motion_complete:
            self.step = 6
            self.step_timer = 0.0

        # Step 6: Scan phase
        elif self.step == 6 and self.step_timer >= 0.5:
            self.publish_status('SCANNING')
            self.move_to_joint_positions(
                [0.5, -0.2, 0.0, -2.0, 0.0, 1.8, 0.785],
                'scan position'
            )
            self.step = 7
            self.step_timer = 0.0

        # Step 7: Wait for scan motion then trigger QR
        elif self.step == 7 and self.motion_complete:
            trigger = String()
            trigger.data = 'SCAN_NOW'
            self.scan_trigger_pub.publish(trigger)
            self.get_logger().info('At scan position! QR trigger sent...')
            self.waiting_for_qr = True
            self.step = 8
            self.step_timer = 0.0

        # Step 8: Wait for QR result (max 5 seconds)
        elif self.step == 8:
            if not self.waiting_for_qr or self.step_timer >= 5.0:
                self.step = 9
                self.step_timer = 0.0

        # Step 9: Place phase based on QR result
        elif self.step == 9 and self.step_timer >= 0.5:
            if self.qr_result == 'A':
                self.publish_status('PLACING_A')
                self.get_logger().info('QR=A → Moving to BIN A (left side)')
                self.move_to_joint_positions(
                    [0.8, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785],
                    'bin A position'
                )
            else:
                self.publish_status('PLACING_B')
                self.get_logger().info('QR=B → Moving to BIN B (right side)')
                self.move_to_joint_positions(
                    [-0.8, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785],
                    'bin B position'
                )
            self.step = 10
            self.step_timer = 0.0

        # Step 10: Wait for place motion
        elif self.step == 10 and self.motion_complete:
            self.get_logger().info('Object placed!')
            self.step = 11
            self.step_timer = 0.0

        # Step 11: Return home
        elif self.step == 11 and self.step_timer >= 1.0:
            self.publish_status('RETURNING_HOME')
            self.move_to_joint_positions(
                [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
                'home position'
            )
            self.step = 12
            self.step_timer = 0.0

        # Step 12: Pipeline complete
        elif self.step == 12 and self.motion_complete:
            self.publish_status('COMPLETE')
            self.get_logger().info('=== Pipeline Complete ===')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
