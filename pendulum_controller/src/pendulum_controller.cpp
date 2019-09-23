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

ControllerNode::ControllerNode(const std::string & node_name,
        std::unique_ptr<Controller> controller,
        std::chrono::nanoseconds update_period,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(node_name, options), update_period_(update_period),
  controller_(std::move(controller))
{

}

void ControllerNode::on_sensor_message(const pendulum_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "position: %f", msg->position);
    controller_->update_sensor_data(*msg);
}

void ControllerNode::on_pendulum_setpoint(const pendulum_msgs::msg::JointCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "position: %f", msg->position);
    controller_->update_setpoint_data(*msg);
}

void ControllerNode::control_timer_callback()
{
    command_message_.position = controller_->compute_output();
    command_pub_->publish(command_message_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerNode::on_configure(const rclcpp_lifecycle::State &)
    {
        sub_sensor_ = this->create_subscription<pendulum_msgs::msg::JointState>(
                "pendulum_sensor", 10, std::bind(&ControllerNode::on_sensor_message, this, std::placeholders::_1));

        // Initialize the publisher for the command message.
        command_pub_ = this->create_publisher<pendulum_msgs::msg::JointCommand>(
                "pendulum_command", 10);

        setpoint_sub_ = this->create_subscription<pendulum_msgs::msg::JointCommand>(
                "pendulum_setpoint", 10, std::bind(&ControllerNode::on_pendulum_setpoint, this, std::placeholders::_1));

        // Initialize the logger publisher.
        logger_pub_ = this->create_publisher<pendulum_msgs::msg::RttestResults>(
                "pendulum_statistics", 10);

        timer_ = this->create_wall_timer(update_period_, std::bind(&ControllerNode::control_timer_callback, this));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerNode::on_activate(const rclcpp_lifecycle::State &)
    {
        command_pub_->on_activate();
        logger_pub_->on_activate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
    {
        command_pub_->on_deactivate();
        logger_pub_->on_deactivate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        command_pub_.reset();
        logger_pub_.reset();
        sub_sensor_.reset();
        setpoint_sub_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        command_pub_.reset();
        logger_pub_.reset();
        sub_sensor_.reset();
        setpoint_sub_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}  // namespace pendulum_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::ControllerNode)