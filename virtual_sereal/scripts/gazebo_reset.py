#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math
import os

class GazeboReset(Node):
    def __init__(self):
        super().__init__('gazebo_reset_node')
        self.reset_service = '/reset_simulation'
        self.client = self.create_client(Empty, self.reset_service)

        self.get_logger().info(f'Waiting for {self.reset_service} service...')
        self.client.wait_for_service()
        self.call_reset_service()

    def call_reset_service(self):
        req = Empty.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'{self.reset_service} called successfully!')
        else:
            self.get_logger().error('Failed to call {self.reset_service}.')

class IMUControllerNode(Node):
    def __init__(self):
        super().__init__('imu_controller_node')

        # IMU 토픽 구독
        self.subscription = self.create_subscription(
            Imu,
            '/imu_sensor_plugin/out',
            self.imu_callback,
            10
        )
        
        # cmd_vel Publisher
        self.cmd_pub = self.create_publisher(Twist, '/demo/cmd_demo', 10)

        # 리셋 클라이언트
        self.reset_service = '/reset_simulation'
        self.reset_client = self.create_client(Empty, self.reset_service)
        self.reset_client.wait_for_service()

        # 안정/중간 상태 범위
        self.stable_lower_limit = -10.0
        self.stable_upper_limit = 20.0
        self.mid_lower_limit = -54.0
        self.mid_upper_limit = 54.0

        self.current_roll = 0.0

        # 0.5초 주기로 상태 체크
        self.control_timer = self.create_timer(0.5, self.control_loop)

        # 50초 주기로 리셋
        self.frame_timer = self.create_timer(50.0, self.reset_simulation_frame)

        # 추가: 무한 반복적으로 주기적인 명령을 내리는 타이머 (예: 1초마다)
        # 이 타이머는 기본적으로 특정 각도 범위에 상관 없이 일정한 명령을 반복적으로 보냅니다.
        # 이를 통해 reset 이후에도 지속적으로 force를 주며 시도하게 합니다.
        self.publish_timer = self.create_timer(1.0, self.periodic_force)

        self.get_logger().info('IMU Controller Node has been started.')

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_roll = math.degrees(roll)

    def control_loop(self):
        angle = self.current_roll

        # 안정 상태 (-10° ~ 20°)
        if self.stable_lower_limit <= angle <= self.stable_upper_limit:
            self.get_logger().info(
                f'Roll {angle:.2f}° within stable range. Stopping wheels.'
            )
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
            return

        # 중간 상태 범위
        # -54° ≤ angle < -10°
        if self.mid_lower_limit <= angle < self.stable_lower_limit:
            # 왼쪽으로 기울었으니 뒤로 가는 힘 (linear.x < 0)
            twist_msg = Twist()
            twist_msg.linear.x = -0.2
            self.cmd_pub.publish(twist_msg)
            self.get_logger().info(
                f'Roll {angle:.2f}° in [-54, -10). Moving backward to correct posture.'
            )
            return

        # 20° < angle ≤ 54°
        if self.stable_upper_limit < angle <= self.mid_upper_limit:
            # 오른쪽으로 기울었으니 앞으로 가는 힘 (linear.x > 0)
            twist_msg = Twist()
            twist_msg.linear.x = 0.2
            self.cmd_pub.publish(twist_msg)
            self.get_logger().info(
                f'Roll {angle:.2f}° in (20, 54]. Moving forward to correct posture.'
            )
            return

        # 그 외 범위: angle < -54° 또는 angle > 54°
        # 복구 불가능하므로 reset
        self.get_logger().warn(
            f'Roll {angle:.2f}° is beyond recoverable range. Resetting simulation.'
        )
        self.call_reset_service()

    def call_reset_service(self):
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Simulation reset due to excessive angle.')
        else:
            self.get_logger().error('Failed to reset simulation due to excessive angle.')

    def reset_simulation_frame(self):
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Simulation reset after 50 seconds!')
        else:
            self.get_logger().error('Failed to reset simulation after 50 seconds.')

    def periodic_force(self):
        # 주기적으로 약한 힘을 주는 코드: 예를 들어 번갈아가며 0.08, 0.11를 주는 식으로 동작할 수 있음
        # 여기서는 간단히 0.08로만 주기적으로 보내도록 함.
        twist_msg = Twist()
        twist_msg.linear.x = 0.08
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info("Applying periodic forward force (0.08) to maintain posture.")


def main(args=None):
    rclpy.init(args=args)
    
    reset_node = GazeboReset()
    reset_node.destroy_node()

    imu_controller_node = IMUControllerNode()
    try:
        rclpy.spin(imu_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
