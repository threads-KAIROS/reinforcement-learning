#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import EntityState
import math
import time


class SetModelState(Node):
    def __init__(self):
        super().__init__('set_model_state')
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service /gazebo/set_entity_state not available, waiting...')
        self.get_logger().info('Service /gazebo/set_entity_state is ready.')

        # Initialize the roll angle
        self.roll_angle = 0.0

        # Call the periodic update function
        self.timer = self.create_timer(2.0, self.update_model)  # Update every 2 seconds

    def update_model(self):
        # Update the roll angle to 1.57 radians
        self.roll_angle = 1.57

        req = SetEntityState.Request()

        # Define the model's name
        req.state = EntityState()
        req.state.name = 'model'

        # Define the desired pose with updated roll value
        req.state.pose = Pose()
        req.state.pose.position.x = 0.0
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.1

        # Convert roll, pitch, yaw to quaternion
        req.state.pose.orientation = self.euler_to_quaternion(self.roll_angle, 0.0, 0.0)

        # Define the desired twist (velocity)
        req.state.twist = Twist()
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        # Make the service call
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully updated the model state with roll=1.57.')
            else:
                self.get_logger().info(f'Failed to update model state: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to a quaternion.
        roll, pitch, yaw are given in radians.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
             math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        quaternion = Pose()
        quaternion.orientation.x = qx
        quaternion.orientation.y = qy
        quaternion.orientation.z = qz
        quaternion.orientation.w = qw
        return quaternion.orientation


def main(args=None):
    rclpy.init(args=args)
    node = SetModelState()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
