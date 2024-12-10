cmake_minimum_required(VERSION 3.8)
project(virtual_sereal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
# find_package(tf_transformations REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)
install(
    DIRECTORY urdf meshes models
    DESTINATION share/${PROJECT_NAME}/
)

# Python 실행 파일 설치
install(PROGRAMS
  scripts/twist_subscriber.py  # 여기에 실행할 스크립트 경로를 작성
  DESTINATION lib/${PROJECT_NAME}  # 실행 파일이 설치될 위치
)

# Python 실행 파일 설치
install(PROGRAMS
  scripts/set_model_state.py  # 여기에 실행할 스크립트 경로를 작성
  DESTINATION lib/${PROJECT_NAME}  # 실행 파일이 설치될 위치
)

# Python 실행 파일 설치
install(PROGRAMS
  scripts/robot_standup.py  # 여기에 실행할 스크립트 경로를 작성
  DESTINATION lib/${PROJECT_NAME}  # 실행 파일이 설치될 위치
)

# Python 실행 파일 설치
install(PROGRAMS
  scripts/set_roll.py  # 여기에 실행할 스크립트 경로를 작성
  DESTINATION lib/${PROJECT_NAME}  # 실행 파일이 설치될 위치
)

# Python 실행 파일 설치
install(PROGRAMS
  scripts/velocity_command_node.py  # 여기에 실행할 스크립트 경로를 작성
  DESTINATION lib/${PROJECT_NAME}  # 실행 파일이 설치될 위치
)

# Python 실행 파일 설치
install(PROGRAMS
  scripts/imu_listener_node.py  # 여기에 실행할 스크립트 경로를 작성
  DESTINATION lib/${PROJECT_NAME}  # 실행 파일이 설치될 위치
)

# Python 실행 파일 설치
install(PROGRAMS
  scripts/gazebo_reset.py  # 여기에 실행할 스크립트 경로를 작성
  DESTINATION lib/${PROJECT_NAME}  # 실행 파일이 설치될 위치
)

ament_package()
