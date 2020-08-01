// Copyright 2019 Carlos San Vicente
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

#include <string>
#include <memory>
#include <utility>

#include "pendulum_controller/pendulum_controller_node.hpp"

namespace pendulum
{
namespace pendulum_controller
{
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

PendulumControllerNode::PendulumControllerNode(
  std::unique_ptr<PendulumController> controller,
  PendulumControllerOptions controller_options,
  const rclcpp::NodeOptions & options =
  rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(controller_options.node_name, options),
  controller_(std::move(controller)),
  controller_options_(controller_options)
{
}

void PendulumControllerNode::on_sensor_message(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  controller_->update_status_data(*msg);
}

void PendulumControllerNode::on_pendulum_setpoint(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  controller_->update_setpoint_data(*msg);
}

void PendulumControllerNode::control_timer_callback()
{
  controller_->update_command_data(command_message_);
  command_pub_->publish(command_message_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
  // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
  // message pool is determined by the number of threads (the maximum number of concurrent accesses
  // to the subscription).
  // Commented because sensor_msgs::msg::JointState is not a fix size msg type
  // auto state_msg_strategy =
  //   std::make_shared<MessagePoolMemoryStrategy<sensor_msgs::msg::JointState, 1>>();
  auto setpoint_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  this->get_state_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      // Do nothing
    };

  state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", controller_options_.status_qos_profile,
    std::bind(
      &PendulumControllerNode::on_sensor_message,
      this, std::placeholders::_1),
    sensor_subscription_options_);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      // Do nothing
    };

  // Initialize the publisher for the command message.
  command_pub_ = this->create_publisher<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_command",
    controller_options_.command_qos_profile,
    command_publisher_options_);

  setpoint_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_setpoint", controller_options_.setpoint_qos_profile,
    std::bind(
      &PendulumControllerNode::on_pendulum_setpoint,
      this, std::placeholders::_1),
    rclcpp::SubscriptionOptions(),
    setpoint_msg_strategy);

  command_timer_ =
    this->create_wall_timer(
    controller_options_.command_publish_period,
    std::bind(&PendulumControllerNode::control_timer_callback, this));
  // cancel immediately to prevent triggering it in this state
  command_timer_->cancel();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  command_pub_->on_activate();
  command_timer_->reset();

  // reset internal state of the controller for a clean start
  controller_->reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  command_timer_->cancel();
  command_pub_->on_deactivate();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  command_timer_.reset();
  command_pub_.reset();
  state_sub_.reset();
  setpoint_sub_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  command_timer_.reset();
  command_pub_.reset();
  state_sub_.reset();
  setpoint_sub_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace pendulum_controller
}  // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::pendulum_controller::PendulumControllerNode)
