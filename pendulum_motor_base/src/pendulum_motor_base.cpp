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

}


void MotorBase::on_command_received (const pendulum_msgs::msg::JointCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Command: %f", msg->position);

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MotorBase::on_configure(const rclcpp_lifecycle::State &)
{
    sensor_pub = this->create_publisher<pendulum_msgs::msg::JointState>("pendulum_sensor", 1);

    command_sub = this->create_subscription<pendulum_msgs::msg::JointCommand>(
            "pendulum_command", 1, std::bind(&MotorBase::on_command_received, this, std::placeholders::_1));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MotorBase::on_activate(const rclcpp_lifecycle::State &)
{
    sensor_pub->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MotorBase::on_deactivate(const rclcpp_lifecycle::State &)
{
    sensor_pub->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MotorBase::on_cleanup(const rclcpp_lifecycle::State &)
{
    //timer_.reset();
    command_sub.reset();
    sensor_pub.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MotorBase::on_shutdown(const rclcpp_lifecycle::State &)
{
    //timer_.reset();
    command_sub.reset();
    sensor_pub.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace pendulum_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::MotorBase)