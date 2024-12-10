#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # URDF file path
    urdf_path = os.path.join(
        get_package_share_directory('virtual_sereal'),
        'urdf',
        'sereal.urdf'
    )
    print(f"URDF file path: {urdf_path}")

    # Read the URDF file
    try:
        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        raise FileNotFoundError(f"URDF file not found at {urdf_path}")
    except IOError as e:
        raise IOError(f"Error reading URDF file: {e}")

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Set true to use simulation time in Gazebo, false otherwise'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
    ])