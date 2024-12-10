#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
import time

class RobotStandup(Node):
    def __init__(self):
        super().__init__('robot_standup')
        self.cli = self.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service /gazebo/apply_body_wrench not available, waiting...')
        self.get_logger().info('Service /gazebo/apply_body_wrench is ready.')

        # Call the service to balance the robot
        self.balance_robot()
        self.set_model_state()

    def balance_robot(self):
        # Apply a small torque to stabilize the robot
        req = ApplyBodyWrench.Request()
        req.body_name = 'sereal::base_link'
        req.reference_frame = 'world'

        # Wrench values (force and torque)
        req.wrench = Wrench()
        req.wrench.force.x = 0.0
        req.wrench.force.y = 0.0
        req.wrench.force.z = 0.0  # No force applied
        req.wrench.torque.x = 0.0
        req.wrench.torque.y = 0.5  # Apply torque around Y-axis to balance
        req.wrench.torque.z = 0.0

        # Duration for the wrench application
        req.duration.sec = 0  # Apply wrench continuously
        req.duration.nanosec = 500000000  # 0.5 seconds

        # Make the service call
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully applied wrench to balance the robot.')
            else:
                self.get_logger().info(f'Failed to apply wrench: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def set_model_state(self):
        req = ApplyBodyWrench.Request()
        req.body_name = 'sereal::base_link'
        req.reference_frame = 'world'
        req.wrench.torque.y = 0.5
        req.duration.sec = 0
        req.duration.nanosec = 500000000

        self.get_logger().info('Preparing to call /gazebo/apply_body_wrench with request...')
        self.get_logger().info(f'Request body_name: {req.body_name}, reference_frame: {req.reference_frame}')

        # 서비스 호출
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStandup()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
