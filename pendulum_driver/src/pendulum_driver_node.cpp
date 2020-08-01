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

#include "pendulum_driver/pendulum_driver_node.hpp"

namespace pendulum
{
namespace pendulum_driver
{
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

PendulumDriverNode::PendulumDriverNode(
  std::unique_ptr<PendulumDriver> driver_interface,
  PendulumDriverOptions driver_options,
  const rclcpp::NodeOptions & options =
  rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(driver_options.node_name, options),
  driver_(std::move(driver_interface)),
  driver_options_(driver_options)
{
  // Initiliaze joint message
  state_message_.name.push_back("cart_base_joint");
  state_message_.position.push_back(0.0);
  state_message_.velocity.push_back(0.0);
  state_message_.effort.push_back(0.0);

  state_message_.name.push_back("pole_joint");
  state_message_.position.push_back(0.0);
  state_message_.velocity.push_back(0.0);
  state_message_.effort.push_back(0.0);
}

void PendulumDriverNode::on_command_received(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  driver_->update_command_data(*msg);
}

void PendulumDriverNode::on_disturbance_received(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  driver_->update_disturbance_data(*msg);
}

void PendulumDriverNode::state_timer_callback()
{
  driver_->update();
  driver_->update_status_data(state_message_);
  state_message_.header.stamp = this->get_clock()->now();
  state_pub_->publish(state_message_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
  // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
  // message pool is determined by the number of threads (the maximum number of concurrent accesses
  // to the subscription).
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  this->get_state_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      // do nothing
    };
  state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", driver_options_.status_qos_profile, sensor_publisher_options_);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      // do nothing
    };
  command_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_command", driver_options_.status_qos_profile,
    std::bind(
      &PendulumDriverNode::on_command_received,
      this, std::placeholders::_1),
    command_subscription_options_,
    command_msg_strategy);

  auto disturbance_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  disturbance_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_disturbance", rclcpp::QoS(1),
    std::bind(
      &PendulumDriverNode::on_disturbance_received,
      this, std::placeholders::_1),
    rclcpp::SubscriptionOptions(),
    disturbance_msg_strategy);

  state_timer_ =
    this->create_wall_timer(
    driver_options_.status_publish_period,
    std::bind(&PendulumDriverNode::state_timer_callback, this));
  // cancel immediately to prevent triggering it in this state
  state_timer_->cancel();

  driver_->init();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  state_pub_->on_activate();
  state_timer_->reset();
  // reset internal state of the driver for a clean start
  driver_->start();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  driver_->stop();
  state_timer_->cancel();
  state_pub_->on_deactivate();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  driver_->shutdown();
  state_timer_.reset();
  state_pub_.reset();
  command_sub_.reset();
  disturbance_sub_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  state_timer_.reset();
  state_pub_.reset();
  command_sub_.reset();
  disturbance_sub_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace pendulum_driver
}  // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::pendulum_driver::PendulumDriverNode)
