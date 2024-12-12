# reinforcement-learning
1. 기존 ros 파일 형식대로 virtual_sereal\launch\spawn_sereal.launch.py을 ros2 launch로 실행
2. virtual_sereal\worlds\empty_world.world line 4~21에서 joint값을 읽어오는 plugin을 추가
3. virtual_sereal\worlds\empty_world.world line 37~40에서 컴퓨터 성능의 한계로 렌더링 옵션 수정
4. virtual_sereal\scripts\rlcode.py를 파이썬으로 실행
5. -1~1의 값의 범위를 step마다 /demo/cmd_demo노드의 linear.x에 줌
6. 기존의 /imu_sensor_plugin/out 노드의 값을 읽어옴
7. line 11-87 강화학습 환경 설정
8. line 92부터 stable_baseline3을 활용한 강화학습 모델 설정
