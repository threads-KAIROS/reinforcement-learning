#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

class GazeboReset(Node):
    def __init__(self):
        super().__init__('gazebo_reset_node')
        self.reset_service = '/reset_simulation'  # 또는 '/reset_world'
        self.client = self.create_client(Empty, self.reset_service)

        # 서비스가 사용 가능할 때까지 대기
        self.get_logger().info(f'Waiting for {self.reset_service} service...')
        self.client.wait_for_service()

        # 서비스 호출
        self.call_reset_service()

    def call_reset_service(self):
        req = Empty.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'{self.reset_service} called successfully!')
        else:
            self.get_logger().error(f'Failed to call {self.reset_service}.')

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
        
        # 힘을 발행할 Publisher
        self.cmd_pub = self.create_publisher(Twist, '/demo/cmd_demo', 10)

        # 리셋 서비스를 위한 클라이언트 생성
        self.reset_service = '/reset_simulation'
        self.reset_client = self.create_client(Empty, self.reset_service)
        self.reset_client.wait_for_service()

        # 각도 범위 설정: roll 값이 -53° ~ 69° 사이일 때는 안정 상태로 간주
        self.upper_limit = 69.0   # degrees
        self.lower_limit = -53.0  # degrees

        # 현재 roll 값
        self.current_roll = 0.0

        # 일정한 힘 크기 설정 (증가 없음)
        self.base_force = 0.5

        # 0.5초 주기로 상태 체크
        self.control_timer = self.create_timer(0.5, self.control_loop)

        # 50초 후에 다시 frame 반복 (reset)
        self.frame_timer = self.create_timer(50.0, self.reset_simulation_frame)

        self.get_logger().info('IMU Controller Node has been started.')

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_roll = math.degrees(roll)

    def control_loop(self):
        # 현재 roll이 지정한 범위 내에 있는지 확인
        if self.lower_limit <= self.current_roll <= self.upper_limit:
            # 범위 안에 있으면 힘 적용 필요 없음.
            self.get_logger().info(
                f'Roll {self.current_roll:.2f}° within range [{self.lower_limit}, {self.upper_limit}]. No force applied.'
            )
            return
        else:
            # 범위를 벗어났으므로 반대 방향으로 일정한 힘을 적용
            twist_msg = Twist()
            if self.current_roll > self.upper_limit:
                # 오른쪽으로 기울었을 때: 일정한 x 방향 힘(positive x)
                twist_msg.linear.x = -self.base_force
            elif self.current_roll < self.lower_limit:
                # 왼쪽으로 기울었을 때: 일정한 -x 방향 힘(negative x)
                twist_msg.linear.x = self.base_force

            self.cmd_pub.publish(twist_msg)
            self.get_logger().info(
                f'Roll {self.current_roll:.2f}° out of range. Applying constant force {twist_msg.linear.x:.4f} to correct posture.'
            )

    def reset_simulation_frame(self):
        # 50초마다 시뮬레이션 리셋
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Simulation reset after 50 seconds!')
        else:
            self.get_logger().error('Failed to reset simulation after 50 seconds.')

def main(args=None):
    rclpy.init(args=args)
    
    # Gazebo 초기화 노드 실행
    reset_node = GazeboReset()
    reset_node.destroy_node()

    # IMU 기반 제어 노드 실행
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
