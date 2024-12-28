launch 파일로 가제보 환경에서 로봇을 불러올 수 있다.

```bash
colcon build --packages-select virtual_sereal --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch virtual_sereal spawn_sereal.launch.py
```

