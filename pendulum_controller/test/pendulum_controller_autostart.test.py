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

import launch
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

import pytest


@pytest.mark.rostest
def generate_test_description():
    package_dir = FindPackageShare('pendulum_controller').find('pendulum_controller')
    param_file_path = os.path.join(package_dir, 'params', 'test.param.yaml')
    param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    controller_node = launch_ros.actions.LifecycleNode(
        package='pendulum_controller',
        executable='pendulum_controller_exe',
        name='pendulum_controller',
        parameters=[param_file],
        arguments=['--autostart', 'True'],
        additional_env={'PYTHONUNBUFFERED': '1'}
    )

    shutdown_timer = launch.actions.TimerAction(
        period=5.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch.events.process.SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda proc: proc is controller_node
                )
            )
        ]
    )

    return (
        launch.LaunchDescription([
            controller_node,
            shutdown_timer,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'controller_node': controller_node,
        }
    )


class TestControllerNode(unittest.TestCase):

    def test_proc_starts(self, proc_info, controller_node):
        proc_info.assertWaitForStartup(process=controller_node, timeout=5)

    def test_proc_terminates(self, proc_info, controller_node):
        proc_info.assertWaitForShutdown(controller_node, timeout=10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_node_graceful_shutdown(self, proc_info, controller_node):
        """Test controller_node graceful shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info, process=controller_node)
