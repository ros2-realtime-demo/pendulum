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
    with_rviz_param = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
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

    ld.add_action(with_rviz_param)
    ld.add_action(robot_state_publisher_runner)
    ld.add_action(rviz_runner)
    ld.add_action(pendulum_state_publisher_runner)

    return ld
