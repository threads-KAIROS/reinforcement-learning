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
        # 비동기 완료 콜백 추가
        def reset_done_cb(fut):
            if fut.result() is not None:
                self.get_logger().info(f'{self.reset_service} called successfully!')
            else:
                self.get_logger().error('Failed to call {self.reset_service}.')

        future.add_done_callback(reset_done_cb)

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
        self.stable_lower_limit = -16.0
        self.stable_upper_limit = 20.0
        self.mid_lower_limit = -48.0
        self.mid_upper_limit = 69.0   # 복구 가능 범위 상한을 69도로 변경

        self.current_roll = 0.0

        # 주기적 타이머
        self.control_timer = self.create_timer(0.5, self.control_loop)
        self.frame_timer = self.create_timer(50.0, self.reset_simulation_frame)
        self.publish_timer = self.create_timer(1.0, self.periodic_force)

        self.get_logger().info('IMU Controller Node has been started.')

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_roll = math.degrees(roll)

    def control_loop(self):
        angle = self.current_roll

        # 모든 조건을 if/elif/else로 처리하고 중간에 return하지 않음
        if self.stable_lower_limit <= angle <= self.stable_upper_limit:
            # 안정 상태 (-10° ~ 20°)
            self.get_logger().info(
                f'Roll {angle:.2f}° within stable range. Stopping wheels.'
            )
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)

        elif self.mid_lower_limit <= angle < self.stable_lower_limit:
            # 중간 상태 범위: -54° ≤ angle < -10°
            # 왼쪽으로 기울었으니 뒤로 가는 힘
            twist_msg = Twist()
            twist_msg.linear.x = -0.2
            self.cmd_pub.publish(twist_msg)
            self.get_logger().info(
                f'Roll {angle:.2f}° in [-54, -10). Moving backward to correct posture.'
            )

        elif self.stable_upper_limit < angle <= self.mid_upper_limit:
            # 중간 상태 범위: 20° < angle ≤ 69°
            # 오른쪽으로 기울었으니 앞으로 가는 힘
            twist_msg = Twist()
            twist_msg.linear.x = 0.21
            self.cmd_pub.publish(twist_msg)
            self.get_logger().info(
                f'Roll {angle:.2f}° in (20, 69]. Moving forward to correct posture.'
            )

        else:
            # 그 외 범위: angle < -54° 또는 angle > 69°
            self.get_logger().warn(
                f'Roll {angle:.2f}° is beyond recoverable range (-54° or >69°). Resetting simulation.'
            )
            self.call_reset_service()
        # 함수 끝까지 실행됨(중간 return 없음)

    def call_reset_service(self):
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        # 비동기 완료 콜백 추가
        def reset_done_cb(fut):
            if fut.result() is not None:
                self.get_logger().info('Simulation reset due to excessive angle.')
            else:
                self.get_logger().error('Failed to reset simulation due to excessive angle.')
        future.add_done_callback(reset_done_cb)

    def reset_simulation_frame(self):
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        def frame_reset_done_cb(fut):
            if fut.result() is not None:
                self.get_logger().info('Simulation reset after 50 seconds!')
            else:
                self.get_logger().error('Failed to reset simulation after 50 seconds.')
        future.add_done_callback(frame_reset_done_cb)

    def periodic_force(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.08
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info("Applying periodic forward force (0.08) to maintain posture.")


def main(args=None):
    rclpy.init(args=args)
    
    # 무한 반복으로 reset 후 IMUControllerNode를 다시 실행
    # shutdown() 호출 제거
    while True:
        reset_node = GazeboReset()
        imu_controller_node = IMUControllerNode()
        try:
            rclpy.spin(imu_controller_node)
        except KeyboardInterrupt:
            break
        finally:
            pass

if __name__ == '__main__':
    main()
