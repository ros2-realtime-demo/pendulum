// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pendulum_controller/pendulum_motor_base.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rttest/utils.h"

namespace pendulum
{
    
    MotorBase::MotorBase(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("motor_node", options)
{
    // Initialize the publisher for the sensor message (the current position of the pendulum).
    sensor_pub = this->create_publisher<pendulum_msgs::msg::JointState>("pendulum_sensor", 10);


    // Create a lambda function to invoke the motor callback when a command is received.
    auto motor_subscribe_callback =
            [&pendulum_motor](pendulum_msgs::msg::JointCommand::ConstSharedPtr msg) -> void
            {
                // TODO: implement on_command_message function
                pendulum_motor->on_command_message(msg);
            };

    // Initialize the subscription to the command message.
    command_sub = this->create_subscription<pendulum_msgs::msg::JointCommand>(
            "pendulum_command", 10, std::bind(&MotorBase::motor_subscribe_callback, this, std::placeholders_1));


    // Notification event topic. All state changes
    // are published here as TransitionEvents with
    // a start and goal state indicating the transition
    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/lc_talker/transition_event",
      10,
      std::bind(&MotorBase::notification_callback, this, std::placeholders::_1));
}

void MotorBase::notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "notify callback: Transition from state %s to %s",
            msg->start_state.label.c_str(), msg->goal_state.label.c_str());
}



}  // namespace pendulum_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::MotorBase)