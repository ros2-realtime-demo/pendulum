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

PendulumDriverNode::PendulumDriverNode(const rclcpp::NodeOptions & options)
: PendulumDriverNode("pendulum_driver", options)
{}

PendulumDriverNode::PendulumDriverNode(
  const std::string & node_name,
  rclcpp::NodeOptions options)
: LifecycleNode(
    node_name.c_str(),
    options),
  sensor_topic_name_(declare_parameter("sensor_topic_name").get<std::string>().c_str()),
  command_topic_name_(declare_parameter("command_topic_name").get<std::string>().c_str()),
  disturbance_topic_name_(declare_parameter("disturbance_topic_name").get<std::string>().c_str()),
  state_publish_period_(std::chrono::microseconds{
      declare_parameter("state_publish_period_us").get<std::uint16_t>()}),
  driver_(
    PendulumDriver::Config(
      declare_parameter("driver.pendulum_mass").get<double>(),
      declare_parameter("driver.cart_mass").get<double>(),
      declare_parameter("driver.pendulum_length").get<double>(),
      declare_parameter("driver.damping_coefficient").get<double>(),
      declare_parameter("driver.gravity").get<double>(),
      declare_parameter("driver.max_cart_force").get<double>(),
      std::chrono::microseconds {
      declare_parameter("driver.physics_update_period").get<std::uint16_t>()
    }
    )
  )
{
  // Initialize joint message
  state_message_.name.push_back("cart_base_joint");
  state_message_.position.push_back(0.0);
  state_message_.velocity.push_back(0.0);
  state_message_.effort.push_back(0.0);

  state_message_.name.push_back("pole_joint");
  state_message_.position.push_back(0.0);
  state_message_.velocity.push_back(0.0);
  state_message_.effort.push_back(0.0);
}


PendulumDriverNode::PendulumDriverNode(
  const std::string & node_name,
  const std::string & sensor_topic_name,
  const std::string & command_topic_name,
  const std::string & disturbance_topic_name,
  std::chrono::microseconds state_publish_period,
  const PendulumDriver::Config & driver_cfg)
: LifecycleNode(node_name.c_str()),
  sensor_topic_name_{sensor_topic_name},
  command_topic_name_{command_topic_name},
  disturbance_topic_name_{disturbance_topic_name},
  state_publish_period_{state_publish_period},
  driver_(driver_cfg)
{
  // Initialize joint message
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
  driver_.update_command_data(*msg);
}

void PendulumDriverNode::on_disturbance_received(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  driver_.update_disturbance_data(*msg);
}

void PendulumDriverNode::state_timer_callback()
{
  driver_.update();
  driver_.update_status_data(state_message_);
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
    sensor_topic_name_.c_str(), rclcpp::QoS(10), sensor_publisher_options_);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      // do nothing
    };
  command_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    command_topic_name_.c_str(), rclcpp::QoS(10),
    std::bind(
      &PendulumDriverNode::on_command_received,
      this, std::placeholders::_1),
    command_subscription_options_,
    command_msg_strategy);

  auto disturbance_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  disturbance_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    disturbance_topic_name_.c_str(), rclcpp::QoS(10),
    std::bind(
      &PendulumDriverNode::on_disturbance_received,
      this, std::placeholders::_1),
    rclcpp::SubscriptionOptions(),
    disturbance_msg_strategy);

  state_timer_ =
    this->create_wall_timer(
    state_publish_period_,
    std::bind(&PendulumDriverNode::state_timer_callback, this));
  // cancel immediately to prevent triggering it in this state
  state_timer_->cancel();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  state_pub_->on_activate();
  state_timer_->reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  state_timer_->cancel();
  state_pub_->on_deactivate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
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
