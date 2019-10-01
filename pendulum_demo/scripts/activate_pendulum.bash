#!/bin/bash
ros2 lifecycle set /pendulum_controller configure
ros2 lifecycle set /pendulum_motor_node configure
ros2 lifecycle set /pendulum_controller activate
ros2 lifecycle set /pendulum_motor_node activate
