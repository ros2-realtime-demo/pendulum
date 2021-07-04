# Copyright 2020 Carlos San Vicente
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
import signal
import unittest

from launch import LaunchDescription
import launch.substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

import pytest


@pytest.mark.rostest
def generate_test_description():
    test_dir = FindPackageShare('pendulum_tests').find('pendulum_tests')

    param_file_path = os.path.join(test_dir, 'params', 'test_autostart.param.yaml')
    param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    controller_launch = Node(
        package='pendulum_controller',
        executable='pendulum_controller_exe',
        output='screen',
        parameters=[param_file],
        arguments=[]
    )

    driver_launch = Node(
        package='pendulum_driver',
        executable='pendulum_driver_exe',
        output='screen',
        parameters=[param_file],
        arguments=[]
    )

    shutdown_timer = launch.actions.TimerAction(
        period=2.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch.events.process.SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda proc: proc is controller_launch
                )
            ),
            launch.actions.EmitEvent(
                event=launch.events.process.SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda proc: proc is driver_launch
                ),
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(controller_launch)
    ld.add_action(driver_launch)
    ld.add_action(shutdown_timer)
    ld.add_action(launch_testing.util.KeepAliveProc())
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld, {
        'controller_launch': controller_launch,
        'driver_launch': driver_launch
    }


class TestPendulumDemo(unittest.TestCase):

    def test_proc_starts(self, proc_info, controller_launch, driver_launch):
        proc_info.assertWaitForStartup(process=controller_launch, timeout=5)
        proc_info.assertWaitForStartup(process=driver_launch, timeout=5)

    def test_proc_terminates(self, proc_info, controller_launch, driver_launch):
        proc_info.assertWaitForShutdown(controller_launch, timeout=10)
        proc_info.assertWaitForShutdown(driver_launch, timeout=10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_node_graceful_shutdown(self, proc_info, controller_launch, driver_launch):
        launch_testing.asserts.assertExitCodes(proc_info, process=controller_launch)
        launch_testing.asserts.assertExitCodes(proc_info, process=driver_launch)
