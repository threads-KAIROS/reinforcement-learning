#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty  # Gazebo 리셋 서비스를 사용하기 위해 추가
# 필요한 경우 IMU 메시지 타입을 임포트하세요
from sensor_msgs.msg import Imu

class VelocityCommandNode(Node):
    def __init__(self):
        super().__init__('velocity_command_node')

        # /velocity_command 토픽을 구독 (속도 명령)
        self.subscription = self.create_subscription(
            Float64,
            '/velocity_command',
            self.velocity_callback,
            10)
        self.subscription  # prevent unused variable warning

        # 기울기 데이터를 구독하는 토픽 설정
        # 실제로 사용하는 메시지 타입과 토픽 이름으로 수정하세요
        self.inclination_subscription = self.create_subscription(
            Float64,
            '/inclination_angle',  # 기울기 데이터 토픽 이름
            self.inclination_callback,
            10)
        self.inclination_subscription  # prevent unused variable warning

        # /demo/cmd_demo 토픽에 퍼블리시
        self.publisher_ = self.create_publisher(Twist, '/demo/cmd_demo', 10)

        # Gazebo 리셋 서비스를 위한 클라이언트 생성
        self.reset_service = '/reset_simulation'  # 또는 '/reset_world'
        self.client = self.create_client(Empty, self.reset_service)

        # 서비스가 사용 가능할 때까지 대기
        self.get_logger().info(f'Waiting for {self.reset_service} service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.reset_service} not available, waiting again...')

        self.get_logger().info('Velocity Command Node has been started.')

        # 초기 속도 설정
        self.current_velocity = 0.0
        self.is_velocity_received = False

    def velocity_callback(self, msg):
        self.current_velocity = msg.data
        self.is_velocity_received = True
        self.get_logger().info(f'Received velocity command: {self.current_velocity}')

    def inclination_callback(self, msg):
        # 기울기 데이터를 수신하여 처리
        inclination = msg.data  # 기울기 값 (예: 도 또는 라디안 단위)
        self.get_logger().info(f'Received inclination angle: {inclination}')

        # 임계값 설정 (예: 기울기가 10도 이상일 때)
        threshold = 10.0  # 임계 기울기 값

        if abs(inclination) >= threshold:
            self.get_logger().info('Inclination threshold exceeded, resetting simulation...')
            self.call_reset_service()
        else:
            if self.is_velocity_received:
                twist = Twist()
                twist.linear.x = self.current_velocity
                twist.angular.z = 0.0  # 필요에 따라 각속도 설정

                self.publisher_.publish(twist)
                self.get_logger().info(f'Published Twist message with linear.x = {self.current_velocity}')
            else:
                self.get_logger().warning('Velocity command not received yet.')

    def call_reset_service(self):
        req = Empty.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.reset_callback)

    def reset_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'{self.reset_service} called successfully!')
            # 시뮬레이션이 리셋되었으므로 필요한 초기화 작업 수행
            self.is_velocity_received = False
            self.current_velocity = 0.0
        except Exception as e:
            self.get_logger().error(f'Failed to call {self.reset_service}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
