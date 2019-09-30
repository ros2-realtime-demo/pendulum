#!/bin/bash
while :
do
	echo "Setpoint: 1.45"
  ros2 topic pub -1 /pendulum_setpoint pendulum_msgs/msg/JointCommand "position: 1.45"
  sleep 1.0
  echo "Setpoint: 1.57"
  ros2 topic pub -1 /pendulum_setpoint pendulum_msgs/msg/JointCommand "position: 1.57"
  sleep 1.0
  echo "Setpoint: 1.65"
  ros2 topic pub -1 /pendulum_setpoint pendulum_msgs/msg/JointCommand "position: 1.65"
  sleep 1.0
done
#
