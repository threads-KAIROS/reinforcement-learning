#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class IMUListenerNode(Node):
    def __init__(self):
        super().__init__('imu_listener_node')

        # Initialize roll, pitch, yaw, and z-position variables
        self.current_roll = None
        self.current_pitch = None
        self.current_yaw = None
        self.robot_z_position = None
        self.latest_imu_data = None

        # Subscribe to IMU data topic
        self.inclination_subscription = self.create_subscription(
            Imu,
            '/imu_sensor_plugin/out',  # IMU data topic name
            self.inclination_callback,
            10)

        # Subscribe to Odometry data topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with the actual odometry topic name if necessary
            self.odom_callback,
            10)

        # Timer to process and print IMU data every 1 second
        self.create_timer(1.0, self.process_and_print_imu_data)

        self.get_logger().info('IMU Listener Node has been started.')

    def odom_callback(self, msg):
        """
        Callback for Odometry data.
        Updates the robot's z-position.
        """
        self.robot_z_position = msg.pose.pose.position.z

    def inclination_callback(self, msg):
        """
        Callback for IMU data.
        Stores the latest IMU data without processing it immediately.
        """
        self.latest_imu_data = msg  # Store the latest IMU data

    def process_and_print_imu_data(self):
        """
        Processes and prints the latest IMU data every 1 second.
        """
        if self.latest_imu_data is not None:
            # Extract orientation and convert to roll, pitch, yaw
            orientation_q = self.latest_imu_data.orientation
            orientation_list = [
                orientation_q.x, 
                orientation_q.y, 
                orientation_q.z, 
                orientation_q.w
            ]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)

            # Convert roll, pitch, and yaw to degrees
            self.current_roll = math.degrees(roll)
            self.current_pitch = math.degrees(pitch)
            self.current_yaw = math.degrees(yaw)

            self.get_logger().info(
                f'Processed IMU Data - Roll: {self.current_roll:.2f}, '
                f'Pitch: {self.current_pitch:.2f}, '
                f'Yaw: {self.current_yaw:.2f}'
            )
        else:
            self.get_logger().info('No IMU data received yet.')

def main(args=None):
    """
    Main function to initialize the ROS 2 node and spin it.
    """
    rclpy.init(args=args)
    node = IMUListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
