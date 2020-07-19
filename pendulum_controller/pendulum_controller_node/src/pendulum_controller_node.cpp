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

#include "pendulum_controller_node/pendulum_controller_node.hpp"

namespace pendulum
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
  controller_options_(controller_options),
  timer_jitter_(controller_options.command_publish_period)
{
}

void PendulumControllerNode::on_sensor_message(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  statistics_message_.sensor_stats.msg_count++;
  controller_->update_status_data(*msg);
}

void PendulumControllerNode::on_pendulum_setpoint(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  statistics_message_.setpoint_stats.msg_count++;
  controller_->update_setpoint_data(*msg);
}

void PendulumControllerNode::control_timer_callback()
{
  timer_jitter_.update();
  statistics_message_.timer_stats.jitter_mean_usec = timer_jitter_.mean();
  statistics_message_.timer_stats.jitter_min_usec = timer_jitter_.min();
  statistics_message_.timer_stats.jitter_max_usec = timer_jitter_.max();
  statistics_message_.timer_stats.jitter_std_usec = std::sqrt(timer_jitter_.variance());
  statistics_message_.command_stats.msg_count++;
  statistics_message_.timer_stats.timer_count++;

  controller_->update_command_data(command_message_);
  command_pub_->publish(command_message_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

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
      statistics_message_.sensor_stats.deadline_misses_count++;
    };

  state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", controller_options_.status_qos_profile,
    std::bind(&PendulumControllerNode::on_sensor_message,
    this, std::placeholders::_1),
    sensor_subscription_options_);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      statistics_message_.command_stats.deadline_misses_count++;
    };

  // Initialize the publisher for the command message.
  command_pub_ = this->create_publisher<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_command",
    controller_options_.command_qos_profile,
    command_publisher_options_);

  setpoint_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_setpoint", controller_options_.setpoint_qos_profile,
    std::bind(&PendulumControllerNode::on_pendulum_setpoint,
    this, std::placeholders::_1),
    rclcpp::SubscriptionOptions(),
    setpoint_msg_strategy);

  command_timer_ =
    this->create_wall_timer(controller_options_.command_publish_period,
      std::bind(&PendulumControllerNode::control_timer_callback, this));
  // cancel immediately to prevent triggering it in this state
  command_timer_->cancel();

  if (controller_options_.enable_statistics) {
    // Initialize the statistics publisher.
    statistics_pub_ = this->create_publisher<pendulum_msgs_v2::msg::ControllerStats>(
      "controller_statistics", 1);
    statistics_timer_ =
      this->create_wall_timer(controller_options_.statistics_publish_period, [this] {
          if (resource_usage_.update(this->get_current_state().label() == "active")) {
            resource_usage_.update_message(statistics_message_.rusage_stats);
            statistics_pub_->publish(statistics_message_);
          }
        });
    // cancel immediately to prevent triggering it in this state
    statistics_timer_->cancel();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  command_pub_->on_activate();
  command_timer_->reset();

  if (controller_options_.enable_statistics) {
    statistics_timer_->reset();
    statistics_pub_->on_activate();
  }

  // we need to save resource usage before active state to know the page faults during
  // real-time execution
  resource_usage_.on_activate();

  // reset internal state of the controller for a clean start
  controller_->reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  resource_usage_.on_deactivate();
  command_timer_->cancel();
  command_pub_->on_deactivate();
  if (controller_options_.enable_statistics) {
    statistics_timer_->cancel();
    statistics_pub_->on_deactivate();
  }

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
  command_timer_.reset();
  command_pub_.reset();
  state_sub_.reset();
  setpoint_sub_.reset();
  if (controller_options_.enable_statistics) {
    statistics_pub_.reset();
    statistics_timer_.reset();
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
  command_timer_.reset();
  command_pub_.reset();
  state_sub_.reset();
  setpoint_sub_.reset();
  if (controller_options_.enable_statistics) {
    statistics_pub_.reset();
    statistics_timer_.reset();
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumControllerNode)
