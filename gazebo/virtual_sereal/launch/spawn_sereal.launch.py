import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the model name from environment variable or default to 'sereal'
    model_folder = os.getenv('MODEL_NAME', 'sereal')  # Default to 'sereal' if MODEL_NAME is not set
    sdf_path = os.path.join(
        get_package_share_directory('virtual_sereal'),
        'models',
        model_folder,
        'model.sdf'
    )
    
    # 커스텀 월드 파일 경로 설정
    world_path = os.path.join(
        get_package_share_directory('virtual_sereal'),
        'worlds',
        'empty_world.world'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')  # Adjusted Z pose for initial balance

    # Add orientation arguments
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    return LaunchDescription([
        # Declare pose arguments
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.1', description='Initial Z position'),

        # Declare orientation arguments
        DeclareLaunchArgument('roll', default_value='0.0', description='Initial roll (rotation around X-axis)'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Initial pitch (rotation around Y-axis)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw (rotation around Z-axis)'),

        # Log pose information
        LogInfo(msg=['Spawning sereal model at: ', 
                     'x=', x_pose, ' y=', y_pose, ' z=', z_pose,
                     ' roll=', roll, ' pitch=', pitch, ' yaw=', yaw]),
        
        # Launch Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Gazebo 서버 실행 (커스텀 월드 파일 사용)
        # ExecuteProcess(
        #     cmd=[
        #         'gzserver',
        #         '--verbose',
        #         world_path,
        #         '-s', 'libgazebo_ros_init.so',
        #         '-s', 'libgazebo_ros_factory.so'
        #     ],
        #     output='screen'
        # ),

        # Launch Gazebo client
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Spawn the entity in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'sereal',
                '-file', sdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw
            ],
            output='screen'
        )
    ])