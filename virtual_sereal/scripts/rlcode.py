import gym
from gym import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty  # ROS 2 서비스 정의 가져오기
import time

class CartPoleEnv(gym.Env, Node):
    def __init__(self):
        # super(CartPoleEnv, self).__init__()
        Node.__init__(self, 'cartpole_env')

        # Action space: linear.x 값을 [-1.0, 1.0] 범위에서 설정
        self.action_space = spaces.Box(
            low=np.array([-1.0]), high=np.array([1.0]), dtype=np.float32
        )

        # Observation space: IMU 데이터를 관측으로 사용
        self.observation_space = spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.inf]),
            high=np.array([np.inf, np.inf, np.inf]),
            dtype=np.float32
        )

        # ROS 2 통신 설정
        self.pub = self.create_publisher(Twist, '/demo/cmd_demo', 10)
        self.sub = self.create_subscription(Imu, '/imu_sensor_plugin/out', self.imu_callback, 10)

        self.current_state = np.zeros(3)  # IMU 데이터 저장 공간
        self.done = False

    def imu_callback(self, msg):
        # IMU 데이터 갱신
        self.current_state[0] = msg.linear_acceleration.x
        self.current_state[1] = msg.angular_velocity.y
        self.current_state[2] = msg.orientation.z

        print("angles: ",self.current_state)
        # Done 조건: 각도가 너무 기울어진 경우
        if self.current_state[1] < -0.006 or self.current_state[1] > 0.008:  # 임계값 #-0.0065758610358306584
            self.done = True
            reward = -100

    def step(self, action):
        # linear.x 값 설정
        linear_x = float(action[0])

        # Twist 메시지로 Gazebo에 명령 전달
        twist = Twist()
        twist.linear.x = linear_x
        self.pub.publish(twist)
        # print(linear_x)

        # time.sleep(0.1)

        # 보상 계산: 각도가 수직에 가까울수록 높은 보상
        reward = 1.0 - abs(self.current_state[1])
        # print(reward)

        # 환경 상태 업데이트
        return self.current_state, reward, self.done, {}

    def reset(self):
        # Gazebo 시뮬레이션 리셋 서비스 호출
        client = self.create_client(Empty, '/reset_simulation')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Reset service not available!')
            return self.current_state

        # 서비스 요청 보내기
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Simulation reset successfully')
        else:
            self.get_logger().error('Failed to reset simulation')

        # 환경 초기화
        self.current_state = np.zeros(3)
        self.done = False
        return self.current_state
        


if __name__ == "__main__":
    # ROS 2 초기화
    rclpy.init()

    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env

    # 환경 초기화
    env = CartPoleEnv()

    # PPO 모델 생성
    model = PPO("MlpPolicy", env, verbose=1)

    try:
        # 반복 학습
        total_iterations = 5000  # 학습 반복 횟수
        timesteps_per_iteration = 10  # 각 반복에서의 타임스텝 수

        for iteration in range(total_iterations):
            print(f"Starting iteration {iteration + 1}/{total_iterations}")
            
            # 학습
            model.learn(total_timesteps=timesteps_per_iteration)

            # 학습된 모델 저장
            model_path = f"/home/user/bootcamp/ppo_cartpole_linear_x_iteration_{iteration + 1}"
            model.save(model_path)
            print(f"Model saved at: {model_path}")

    finally:
        # ROS 2 종료
        rclpy.shutdown()