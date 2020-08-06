# Copyright 2019 Carlos San Vicente
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

import os

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import launch.substitutions


def generate_launch_description():
    # Get the bringup directory
    bringup_dir = FindPackageShare('pendulum_bringup').find('pendulum_bringup')

    # Set robot description parameters
    urdf_file = os.path.join(bringup_dir, 'urdf', 'pendulum.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    # Set parameter file path
    param_file_path = os.path.join(bringup_dir, 'params', 'pendulum.param.yaml')
    param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    # Set rviz config path
    rviz_cfg_path = os.path.join(bringup_dir, 'rviz/pendulum.rviz')

    # Create the launch configuration variables
    autostart_param = DeclareLaunchArgument(
        name='autostart',
        default_value='True',
        description='Automatically start lifecycle nodes')
    priority_param = DeclareLaunchArgument(
        name='priority',
        default_value='0',
        description='Set process priority')
    cpu_affinity_param = DeclareLaunchArgument(
        name='cpu-affinity',
        default_value='0',
        description='Set process CPU affinity')
    with_lock_memory_param = DeclareLaunchArgument(
        name='lock-memory',
        default_value='False',
        description='Lock the process memory')
    lock_memory_size_param = DeclareLaunchArgument(
        name='lock-memory-size',
        default_value='0',
        description='Set lock memory size in MB')
    config_child_threads_param = DeclareLaunchArgument(
        name='config-child-threads',
        default_value='False',
        description='Configure process child threads (typically DDS threads)')
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='False',
        description='Launch RVIZ2 in addition to other nodes'
    )

    # Node definitions
    pendulum_demo_runner = Node(
        package='pendulum_demo',
        executable='pendulum_demo',
        output='screen',
        parameters=[param_file],
        arguments=[
           '--autostart', LaunchConfiguration('autostart'),
           '--priority', LaunchConfiguration('priority'),
           '--cpu-affinity', LaunchConfiguration('cpu-affinity'),
           '--lock-memory', LaunchConfiguration('lock-memory'),
           '--lock-memory-size', LaunchConfiguration('lock-memory-size'),
           '--config-child-threads', LaunchConfiguration('config-child-threads')
           ]
    )

    robot_state_publisher_runner = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    rviz_runner = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    return LaunchDescription([
        autostart_param,
        priority_param,
        cpu_affinity_param,
        with_lock_memory_param,
        lock_memory_size_param,
        config_child_threads_param,
        with_rviz_param,
        robot_state_publisher_runner,
        pendulum_demo_runner,
        rviz_runner
    ])
