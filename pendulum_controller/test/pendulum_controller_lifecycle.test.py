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
import re

import unittest

import launch
import launch.event_handlers.on_process_start

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

import lifecycle_msgs.msg

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
        arguments=['--autostart', 'False'],
        additional_env={'PYTHONUNBUFFERED': '1'},
        namespace=''
    )

    # Right after the talker starts, make it take the 'configure' transition.
    event_configure = launch.actions.RegisterEventHandler(
        launch.event_handlers.on_process_start.OnProcessStart(
            target_action=controller_node,
            on_start=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(controller_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    # When the node reaches the 'inactive' state, make it take the 'activate' transition.
    event_activate = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=controller_node,
            start_state='configuring', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(controller_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    # When the node reaches the 'active' state, wait a bit and then make it take the
    # 'deactivate' transition.
    event_deactivate = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=controller_node, start_state='activating', goal_state='active',
            entities=[
                launch.actions.TimerAction(period=1.0, actions=[
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(controller_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                    )),
                ]),
            ],
        )
    )
    # When the node reaches the 'inactive' state coming from the 'active' state,
    # make it take the 'cleanup' transition.
    event_clean = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=controller_node,
            start_state='deactivating', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(controller_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP,
                )),
            ],
        )
    )
    # When the node reaches the 'unconfigured' state after a 'cleanup' transition,
    # make it take the 'unconfigured_shutdown' transition.
    event_shutdown = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=controller_node,
            start_state='cleaningup', goal_state='unconfigured',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(controller_node),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
                    ),
                )),
            ],
        )
    )

    return (
        launch.LaunchDescription([
            controller_node,
            event_configure,
            event_activate,
            event_deactivate,
            event_clean,
            event_shutdown,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'controller_node': controller_node,
        }
    )


class TestcontrollerNode(unittest.TestCase):

    def test_proc_starts(self, proc_info, controller_node):
        proc_info.assertWaitForStartup(process=controller_node, timeout=5)


class TestLifecycle(unittest.TestCase):

    def test_controller_lifecycle(self, proc_output, controller_node):
        """Test pendulum_controller lifecycle."""
        proc_output.assertWaitFor('Configuring', process=controller_node, timeout=5)
        proc_output.assertWaitFor('Activating', process=controller_node, timeout=5)
        proc_output.assertWaitFor('Deactivating', process=controller_node, timeout=10)
        pattern = re.compile(r'Cart position = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=controller_node, timeout=5)
        pattern = re.compile(r'Cart velocity = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=controller_node, timeout=5)
        pattern = re.compile(r'Pole angular velocity = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=controller_node, timeout=5)
        pattern = re.compile(r'Teleoperation cart position = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=controller_node, timeout=5)
        pattern = re.compile(r'Teleoperation cart velocity = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=controller_node, timeout=5)
        pattern = re.compile(r'Force command = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=controller_node, timeout=5)
        proc_output.assertWaitFor('Cleaning up', process=controller_node, timeout=5)
        proc_output.assertWaitFor('Shutting down', process=controller_node, timeout=5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_node_graceful_shutdown(self, proc_info, controller_node):
        """Test controller_node graceful shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info, process=controller_node)
