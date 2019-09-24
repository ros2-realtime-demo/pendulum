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

#include "pendulum_motor/pendulum_motor_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rttest/utils.h"

namespace pendulum
{

PendulumMotorNode::PendulumMotorNode(const std::string & node_name,
        std::chrono::nanoseconds publish_period,
        std::unique_ptr<PendulumMotor> motor,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  publish_period_(publish_period), motor_(std::move(motor))
{
}

void PendulumMotorNode::on_command_received (const pendulum_msgs::msg::JointCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Command: %f", msg->position);

    motor_->update_motor_command(*msg);
}

void PendulumMotorNode::sensor_timer_callback()
{
    //RCLCPP_INFO(this->get_logger(), "position: %f", command_message_.position);
    sensor_message_.velocity = motor_->get_velocity();
    sensor_message_.position = motor_->get_position();
    sensor_pub_->publish(sensor_message_);
}

void PendulumMotorNode::update_motor_callback()
{
  motor_->update_motor_state();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_configure(const rclcpp_lifecycle::State &)
{
    sensor_pub_ = this->create_publisher<pendulum_msgs::msg::JointState>("pendulum_sensor", 1);

    command_sub_ = this->create_subscription<pendulum_msgs::msg::JointCommand>(
            "pendulum_command", 1, std::bind(&PendulumMotorNode::on_command_received, this, std::placeholders::_1));

    sensor_timer_ = this->create_wall_timer(publish_period_, std::bind(&PendulumMotorNode::sensor_timer_callback, this));
    update_motor_timer_ = this->create_wall_timer(publish_period_, std::bind(&PendulumMotorNode::update_motor_callback, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_activate(const rclcpp_lifecycle::State &)
{
    sensor_pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    sensor_pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    sensor_timer_.reset();
    update_motor_timer_.reset();
    command_sub_.reset();
    sensor_pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    sensor_timer_.reset();
    update_motor_timer_.reset();
    command_sub_.reset();
    sensor_pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace pendulum_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumMotorNode)
