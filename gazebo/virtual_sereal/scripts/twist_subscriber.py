#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('twist_subscriber')  # 노드 이름 설정
        self.subscription = self.create_subscription(
            Twist,  # 메시지 타입
            '/demo/cmd_demo',  # 구독할 토픽 이름
            self.listener_callback,  # 메시지 수신 시 실행할 콜백 함수
            10  # QoS 설정 (버퍼 크기)
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Twist message: Linear x: {msg.linear.x}, '
                               f'Angular z: {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = TwistSubscriber()  # 노드 생성
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS 2 종료


if __name__ == '__main__':
    main()
