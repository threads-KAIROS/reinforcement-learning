#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Quaternion
import sys
import math

class SetRollNode(Node):
    def __init__(self):
        super().__init__('set_roll_node')

        # Parse the roll value from command line arguments
        if len(sys.argv) != 2:
            self.get_logger().error("Usage: ros2 run virtual_sereal set_roll.py <roll_in_radians>")
            sys.exit(1)
        try:
            new_roll = float(sys.argv[1])
        except ValueError:
            self.get_logger().error("Roll value must be a float representing radians.")
            sys.exit(1)

        # Create a client for the '/gazebo/get_entity_state' service
        get_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/get_entity_state service...')

        # Create the request to get the current state
        get_state_request = GetEntityState.Request()
        get_state_request.name = 'sereal'

        # Call the service
        future = get_state_client.call_async(get_state_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            current_state = future.result().state
            self.get_logger().info('Successfully got current state')
        else:
            self.get_logger().error('Failed to get current state')
            sys.exit(1)

        # Create a client for the '/gazebo/set_entity_state' service
        set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        # Prepare the new state
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = 'sereal'
        request.state.pose = current_state.pose  # Use the current pose

        # Update the roll
        # Extract current pitch and yaw
        current_orientation = current_state.pose.orientation
        current_roll, current_pitch, current_yaw = self.quaternion_to_euler(
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w
        )

        # Create a new quaternion with the new roll
        q = self.euler_to_quaternion(new_roll, current_pitch, current_yaw)
        request.state.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Call the service
        future = set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Successfully updated roll to {}'.format(new_roll))
        else:
            self.get_logger().error('Failed to update roll')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles.
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = SetRollNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
