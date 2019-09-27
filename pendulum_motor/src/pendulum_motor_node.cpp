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

#include "pendulum_motor/pendulum_motor_node.hpp"
#include "rttest/utils.h"

using namespace rclcpp_lifecycle::node_interfaces;

namespace pendulum
{
PendulumMotorNode::PendulumMotorNode(const std::string & node_name,
        std::chrono::nanoseconds publish_period,
        std::unique_ptr<PendulumMotor> motor,
        const rclcpp::NodeOptions & options =
         rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  publish_period_(publish_period),
  motor_(std::move(motor))
{
}

void PendulumMotorNode::on_command_received (
  const pendulum_msgs::msg::JointCommand::SharedPtr msg)
{
    motor_->update_command_data(*msg);
}

void PendulumMotorNode::sensor_timer_callback()
{
    //RCLCPP_INFO(this->get_logger(), "position: %f", command_message_.position);
    motor_->update_sensor_data(sensor_message_);
    sensor_pub_->publish(sensor_message_);
}

void PendulumMotorNode::update_motor_callback()
{
  motor_->update();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumMotorNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    std::chrono::milliseconds deadline_duration(1); //TODO: change this period
    rclcpp::QoS qos_deadline_profile(10);
    qos_deadline_profile.deadline(deadline_duration);
    this->get_sensor_options().event_callbacks.deadline_callback =
       [this](rclcpp::QOSDeadlineOfferedInfo & event) -> void
       {
         RCUTILS_LOG_INFO_NAMED(get_name(),
           "Offered deadline missed - total %d delta %d",
           event.total_count, event.total_count_change);
       };
    sensor_pub_ = this->create_publisher<pendulum_msgs::msg::JointState>(
      "pendulum_sensor", qos_deadline_profile, sensor_publisher_options_);

      this->get_command_options().event_callbacks.deadline_callback =
       [this](rclcpp::QOSDeadlineRequestedInfo & event) -> void
       {
         RCUTILS_LOG_INFO_NAMED(get_name(),
           "Requested deadline missed - total %d delta %d",
           event.total_count, event.total_count_change);
       };
    command_sub_ = this->create_subscription<pendulum_msgs::msg::JointCommand>(
            "pendulum_command", qos_deadline_profile,
             std::bind(&PendulumMotorNode::on_command_received, this, std::placeholders::_1),
           command_subscription_options_);

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumMotorNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    sensor_pub_->on_activate();
    sensor_timer_ = this->create_wall_timer(publish_period_, std::bind(&PendulumMotorNode::sensor_timer_callback, this));
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumMotorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    sensor_pub_->on_deactivate();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumMotorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    sensor_timer_.reset();
    update_motor_timer_.reset();
    command_sub_.reset();
    sensor_pub_.reset();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PendulumMotorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
    sensor_timer_.reset();
    update_motor_timer_.reset();
    command_sub_.reset();
    sensor_pub_.reset();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace pendulum_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumMotorNode)
