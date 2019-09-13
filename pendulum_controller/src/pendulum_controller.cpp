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

#include "pendulum_controller/pendulum_controller.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rttest/utils.h"

namespace pendulum
{

Controller::Controller(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("controller_node", options)
{
   sub_sensor_ = this->create_subscription<pendulum_msgs::msg::JointState>(
            "pendulum_sensor", 10, std::bind(&Controller::on_sensor_message, this, std::placeholders::_1));

    // Initialize the publisher for the command message.
    command_pub_ = this->create_publisher<pendulum_msgs::msg::JointCommand>(
            "pendulum_command", 10);

    setpoint_sub_ = this->create_subscription<pendulum_msgs::msg::JointCommand>(
            "pendulum_setpoint", 10, std::bind(&Controller::on_pendulum_setpoint, this, std::placeholders::_1));

    // Initialize the logger publisher.
    logger_pub_ = this->create_publisher<pendulum_msgs::msg::RttestResults>(
            "pendulum_statistics", 10);

    // Notification event topic. All state changes
    // are published here as TransitionEvents with
    // a start and goal state indicating the transition
    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/lc_talker/transition_event",
      10,
      std::bind(&Controller::notification_callback, this, std::placeholders::_1));
}

void Controller::on_sensor_message(const pendulum_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "position: %f", msg->position);
}

void Controller::on_pendulum_setpoint(const pendulum_msgs::msg::JointCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "position: %f", msg->position);

    // just publish again the msg for the moment
    command_pub_->publish(*msg);
}


void Controller::notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "notify callback: Transition from state %s to %s",
            msg->start_state.label.c_str(), msg->goal_state.label.c_str());
}



}  // namespace pendulum_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::Controller)