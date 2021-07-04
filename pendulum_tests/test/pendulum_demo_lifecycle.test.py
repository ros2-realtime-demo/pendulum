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
import pendulum2_msgs.msg

import pytest
import rclpy


def create_lifecycle_events(node):
    event_configure = launch.actions.RegisterEventHandler(
        launch.event_handlers.on_process_start.OnProcessStart(
            target_action=node,
            on_start=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    event_activate = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    event_deactivate = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node, start_state='activating', goal_state='active',
            entities=[
                launch.actions.TimerAction(period=2.0, actions=[
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                    )),
                ]),
            ],
        )
    )
    event_clean = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node,
            start_state='deactivating', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP,
                )),
            ],
        )
    )
    event_shutdown = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node,
            start_state='cleaningup', goal_state='unconfigured',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
                    ),
                )),
            ],
        )
    )
    return ([event_configure,
             event_activate,
             event_deactivate,
             event_clean,
             event_shutdown])


@pytest.mark.rostest
def generate_test_description():
    package_dir = FindPackageShare('pendulum_demo').find('pendulum_demo')
    param_file_path = os.path.join(package_dir, 'params', 'test.param.yaml')
    param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    driver_node = launch_ros.actions.LifecycleNode(
        package='pendulum_driver',
        executable='pendulum_driver_exe',
        name='pendulum_driver',
        parameters=[param_file],
        arguments=['--autostart', 'False'],
        additional_env={'PYTHONUNBUFFERED': '1'},
        namespace=''
    )

    controller_node = launch_ros.actions.LifecycleNode(
        package='pendulum_controller',
        executable='pendulum_controller_exe',
        name='pendulum_controller',
        parameters=[param_file],
        arguments=['--autostart', 'False'],
        additional_env={'PYTHONUNBUFFERED': '1'},
        namespace=''
    )

    ld = launch.LaunchDescription()
    ld.add_action(controller_node)
    ld.add_action(driver_node)
    for ev in create_lifecycle_events(controller_node):
        ld.add_action(ev)
    for ev in create_lifecycle_events(driver_node):
        ld.add_action(ev)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return (ld,
            {
                'driver_node': driver_node,
                'controller_node': controller_node,
            })


class TestDriverNode(unittest.TestCase):

    def test_proc_starts(self, proc_info, driver_node, controller_node):
        proc_info.assertWaitForStartup(process=driver_node, timeout=5)
        proc_info.assertWaitForStartup(process=controller_node, timeout=5)


class TestLifecycle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('teleop_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_controller_node_transmits(self, controller_node, driver_node, proc_output):
        pub = self.node.create_publisher(pendulum2_msgs.msg.PendulumTeleop, 'teleop', 10)

        # Wait until both nodes are activated
        proc_output.assertWaitFor('Configuring', process=driver_node, timeout=5)
        proc_output.assertWaitFor('Configuring', process=controller_node, timeout=5)
        proc_output.assertWaitFor('Activating', process=driver_node, timeout=5)
        proc_output.assertWaitFor('Activating', process=controller_node, timeout=5)

        # Send teleoperation command to move the cart
        msg = pendulum2_msgs.msg.PendulumTeleop()
        msg.cart_position = 10.0
        pub.publish(msg)

        proc_output.assertWaitFor('Deactivating', process=driver_node, timeout=5)
        pattern = re.compile(r'Cart position = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=driver_node, timeout=5)
        pattern = re.compile(r'Cart velocity = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=driver_node, timeout=5)
        # Expect pole angle between 2.7 and 3.6 radians
        pattern = re.compile(r'Pole angle = (2\.[7-9](\d+)?)|(3\.[0-6](\d+)?)')
        proc_output.assertWaitFor(expected_output=pattern, process=driver_node, timeout=5)
        pattern = re.compile(r'Pole angular velocity = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=driver_node, timeout=5)
        pattern = re.compile(r'Controller force command = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=driver_node, timeout=5)
        pattern = re.compile(r'Disturbance force = [+-]?\d+(?:\.\d+)?')
        proc_output.assertWaitFor(expected_output=pattern, process=driver_node, timeout=5)
        proc_output.assertWaitFor('Cleaning up', process=driver_node, timeout=5)
        proc_output.assertWaitFor('Shutting down', process=driver_node, timeout=5)

        self.node.destroy_publisher(pub)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_node_graceful_shutdown(self, proc_info, driver_node, controller_node):
        """Test controller_node graceful shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info, process=driver_node)
        launch_testing.asserts.assertExitCodes(proc_info, process=controller_node)
