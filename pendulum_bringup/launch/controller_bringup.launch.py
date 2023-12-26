# Copyright 2021 Carlos San Vicente
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

    # Set parameter file path
    param_file_path = os.path.join(bringup_dir, 'params', 'pendulum.param.yaml')
    param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    with_controller_param = DeclareLaunchArgument(
        'controller',
        default_value='True',
        description='Launch controller node'
    )

    # Node definitions
    pendulum_controller_runner = Node(
        package='pendulum_controller',
        executable='pendulum_controller_exe',
        output='screen',
        parameters=[param_file],
        arguments=[],
        condition=IfCondition(LaunchConfiguration('controller'))
    )

    ld = LaunchDescription()
    ld.add_action(with_controller_param)
    ld.add_action(pendulum_controller_runner)

    return ld
