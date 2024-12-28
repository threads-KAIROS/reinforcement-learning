```bash
colcon build --packages-select virtual_sereal --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch virtual_sereal spawn_sereal.launch.py
```

