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

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

PendulumDriverNode::PendulumDriverNode(
  std::unique_ptr<PendulumDriverInterface> driver_interface,
  PendulumDriverOptions driver_options,
  const rclcpp::NodeOptions & options =
  rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(driver_options.node_name, options),
  driver_interface_(std::move(driver_interface)),
  driver_options_(driver_options),
  timer_jitter_(driver_options.status_publish_period)
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
  statistics_message_.sensor_stats.msg_count++;
  driver_interface_->update_command_data(*msg);
}

void PendulumDriverNode::on_disturbance_received(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  driver_interface_->update_disturbance_data(*msg);
}

void PendulumDriverNode::state_timer_callback()
{
  timer_jitter_.update();
  statistics_message_.timer_stats.jitter_mean_usec = timer_jitter_.mean();
  statistics_message_.timer_stats.jitter_min_usec = timer_jitter_.min();
  statistics_message_.timer_stats.jitter_max_usec = timer_jitter_.max();
  statistics_message_.timer_stats.jitter_std_usec = std::sqrt(timer_jitter_.variance());
  statistics_message_.command_stats.msg_count++;
  statistics_message_.timer_stats.timer_count++;

  driver_interface_->update_status_data(state_message_);
  state_pub_->publish(state_message_);
}

void PendulumDriverNode::update_driver_callback()
{
  driver_interface_->update();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

  // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
  // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
  // message pool is determined by the number of threads (the maximum number of concurrent accesses
  // to the subscription).
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  this->get_state_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      this->statistics_message_.sensor_stats.deadline_misses_count++;
    };
  state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", driver_options_.status_qos_profile, sensor_publisher_options_);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      this->statistics_message_.command_stats.deadline_misses_count++;
    };
  command_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_command", driver_options_.status_qos_profile,
    std::bind(&PendulumDriverNode::on_command_received,
    this, std::placeholders::_1),
    command_subscription_options_,
    command_msg_strategy);

  auto disturbance_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  disturbance_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_disturbance", rclcpp::QoS(1),
    std::bind(&PendulumDriverNode::on_disturbance_received,
    this, std::placeholders::_1),
    rclcpp::SubscriptionOptions(),
    disturbance_msg_strategy);

  state_timer_ =
    this->create_wall_timer(driver_options_.status_publish_period,
      std::bind(&PendulumDriverNode::state_timer_callback, this));
  // cancel immediately to prevent triggering it in this state
  state_timer_->cancel();

  if (driver_options_.enable_statistics) {
    // Initialize the statistics publisher.
    statistics_pub_ = this->create_publisher<pendulum_msgs_v2::msg::PendulumStats>(
      "driver_statistics", 1);
    statistics_timer_ =
      this->create_wall_timer(driver_options_.statistics_publish_period, [this] {
          if (resource_usage_.update(this->get_current_state().label() == "active")) {
            resource_usage_.update_message(statistics_message_.rusage_stats);
            statistics_pub_->publish(statistics_message_);
          }
        });
    // cancel immediately to prevent triggering it in this state
    statistics_timer_->cancel();
  }

  driver_interface_->init();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  state_pub_->on_activate();
  state_timer_->reset();

  if (driver_options_.enable_statistics) {
    statistics_timer_->reset();
    statistics_pub_->on_activate();
  }

  // we need to save resource usage before active state to know the page faults during
  // real-time execution
  resource_usage_.on_activate();

  // reset internal state of the driver for a clean start
  driver_interface_->start();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  driver_interface_->stop();

  resource_usage_.on_deactivate();
  state_timer_->cancel();
  state_pub_->on_deactivate();
  if (driver_options_.enable_statistics) {
    statistics_timer_->cancel();
    statistics_pub_->on_deactivate();
  }

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
  driver_interface_->shutdown();
  state_timer_.reset();
  state_pub_.reset();
  command_sub_.reset();
  disturbance_sub_.reset();

  if (driver_options_.enable_statistics) {
    statistics_timer_.reset();
    statistics_pub_.reset();
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
  state_timer_.reset();
  state_pub_.reset();
  command_sub_.reset();
  disturbance_sub_.reset();

  if (driver_options_.enable_statistics) {
    statistics_timer_.reset();
    statistics_pub_.reset();
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumDriverNode)
