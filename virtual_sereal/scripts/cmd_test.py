import time
import os

os.system('ros2 topic pub /demo/cmd_demo geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1')
time.sleep(0.05)  # 1초 대기

while True:
    # 첫 번째 명령 실행
    os.system('ros2 topic pub /demo/cmd_demo geometry_msgs/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1')
    time.sleep(0.05)  # 1초 대기

    # 두 번째 명령 실행
    os.system('ros2 topic pub /demo/cmd_demo geometry_msgs/Twist "{linear: {x: -0.005, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1')
    time.sleep(0.05)  # 1초 대기