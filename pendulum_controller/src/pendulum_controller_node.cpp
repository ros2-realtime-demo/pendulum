// Copyright 2019
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

#include "rttest/utils.h"
#include "pendulum_controller/pendulum_controller_node.hpp"

using namespace rclcpp_lifecycle::node_interfaces;

namespace pendulum
{

PendulumControllerNode::PendulumControllerNode(const std::string & node_name,
        std::unique_ptr<PendulumController> controller,
        std::chrono::nanoseconds publish_period,
        const rclcpp::NodeOptions & options =
         rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  publish_period_(publish_period),
  controller_(std::move(controller))
  { }

void PendulumControllerNode::on_sensor_message(
  const pendulum_msgs::msg::JointState::SharedPtr msg)
{
    controller_->write(*msg);
}

void PendulumControllerNode::on_pendulum_setpoint(
  const pendulum_msgs::msg::JointCommand::SharedPtr msg)
{
    controller_->write(*msg);
}

void PendulumControllerNode::control_timer_callback()
{
    controller_->read(command_message_);
    command_pub_->publish(command_message_);
}

const pendulum_msgs::msg::JointCommand &
  PendulumControllerNode::get_next_command_message() const
{
  return command_message_;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
        std::chrono::milliseconds deadline_duration(1);
        rclcpp::QoS qos_deadline_profile(10);
        qos_deadline_profile.deadline(deadline_duration);
        sub_sensor_ = this->create_subscription<pendulum_msgs::msg::JointState>(
                "pendulum_sensor", qos_deadline_profile,
                 std::bind(&PendulumControllerNode::on_sensor_message, this, std::placeholders::_1),
                 sensor_subscription_options_);
       this->get_sensor_options().event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineRequestedInfo & event) -> void
        {
          RCUTILS_LOG_INFO_NAMED(get_name(),
            "Requested deadline missed - total %d delta %d",
            event.total_count, event.total_count_change);
        };

        // Initialize the publisher for the command message.
        command_pub_ = this->create_publisher<pendulum_msgs::msg::JointCommand>(
                "pendulum_command", qos_deadline_profile, command_publisher_options_);

        this->get_command_options().event_callbacks.deadline_callback =
           [this](rclcpp::QOSDeadlineOfferedInfo & event) -> void
           {
             RCUTILS_LOG_INFO_NAMED(get_name(),
               "Offered deadline missed - total %d delta %d",
               event.total_count, event.total_count_change);
           };
        setpoint_sub_ = this->create_subscription<pendulum_msgs::msg::JointCommand>(
                "pendulum_setpoint", 1, std::bind(&PendulumControllerNode::on_pendulum_setpoint, this, std::placeholders::_1));

        // Initialize the logger publisher.
        logger_pub_ = this->create_publisher<pendulum_msgs::msg::RttestResults>(
                "pendulum_statistics", 1);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
        command_pub_->on_activate();
        logger_pub_->on_activate();
        timer_ = this->create_wall_timer(publish_period_, std::bind(&PendulumControllerNode::control_timer_callback, this));
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
        command_pub_->on_deactivate();
        logger_pub_->on_deactivate();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
        timer_.reset();
        command_pub_.reset();
        logger_pub_.reset();
        sub_sensor_.reset();
        setpoint_sub_.reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
        timer_.reset();
        command_pub_.reset();
        logger_pub_.reset();
        sub_sensor_.reset();
        setpoint_sub_.reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace pendulum_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumControllerNode)
