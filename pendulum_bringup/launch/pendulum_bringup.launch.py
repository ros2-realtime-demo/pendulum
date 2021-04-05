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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        'rviz',
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
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    rviz_runner = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    pendulum_state_publisher_runner = Node(
        package='pendulum_state_publisher',
        executable='pendulum_state_publisher',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription()

    ld.add_action(autostart_param)
    ld.add_action(priority_param)
    ld.add_action(cpu_affinity_param)
    ld.add_action(with_lock_memory_param)
    ld.add_action(lock_memory_size_param)
    ld.add_action(config_child_threads_param)
    ld.add_action(with_rviz_param)
    ld.add_action(robot_state_publisher_runner)
    ld.add_action(pendulum_demo_runner)
    ld.add_action(rviz_runner)
    ld.add_action(pendulum_state_publisher_runner)

    return ld
