import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates

class AngularVelocity(Node):
    def __init__(self):
        super().__init__('angular_velocity')
        self.subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.callback,
            10
        )

    def callback(self, msg):
        # 'left_link'의 인덱스 확인
        index = msg.name.index('sereal::left_link')
        angular_velocity = msg.twist[index].angular
        self.get_logger().info(f"Left wheel Angular Velocity: {angular_velocity}")

def main(args=None):
    rclpy.init(args=args)
    node = AngularVelocity()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

