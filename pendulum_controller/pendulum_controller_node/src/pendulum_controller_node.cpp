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

#include "rttest/utils.h"
#include "pendulum_controller_node/pendulum_controller_node.hpp"

namespace pendulum
{

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

PendulumControllerNode::PendulumControllerNode(
  const std::string & node_name,
  std::unique_ptr<PendulumController> controller,
  std::chrono::nanoseconds publish_period,
  const rclcpp::QoS & qos_profile,
  const rclcpp::QoS & setpoint_qos_profile,
  const bool check_memory = false,
  const rclcpp::NodeOptions & options =
  rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  publish_period_(publish_period),
  controller_(std::move(controller)),
  qos_profile_(qos_profile),
  setpoint_qos_profile_(setpoint_qos_profile),
  check_memory_(check_memory)
{
  if (check_memory_) {
  #ifdef PENDULUM_CONTROLLER_MEMORYTOOLS_ENABLED
    osrf_testing_tools_cpp::memory_tools::initialize();
    osrf_testing_tools_cpp::memory_tools::enable_monitoring();
    if (!osrf_testing_tools_cpp::memory_tools::is_working()) {
      throw std::runtime_error(
              "Memory checking does not work properly. Please consult the documentation on how to "
              "properly set it up.");
    }
    const auto on_unexpected_memory =
      [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService & service) {
        // this will cause a backtrace to be printed for each unexpected memory operations
        service.print_backtrace();
      };
    osrf_testing_tools_cpp::memory_tools::on_unexpected_calloc(on_unexpected_memory);
    osrf_testing_tools_cpp::memory_tools::on_unexpected_free(on_unexpected_memory);
    osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(on_unexpected_memory);
    osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(on_unexpected_memory);

  #else
    throw std::runtime_error(
            "OSRF memory tools is not installed. Memory check must be disabled.");
  #endif
  }
}

void PendulumControllerNode::on_sensor_message(
  const pendulum_msgs::msg::JointState::SharedPtr msg)
{
  controller_->update_sensor_data(*msg);
}

void PendulumControllerNode::on_pendulum_setpoint(
  const pendulum_msgs::msg::JointCommand::SharedPtr msg)
{
  controller_->update_setpoint_data(*msg);
}

void PendulumControllerNode::control_timer_callback()
{
  controller_->update_command_data(command_message_);
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

  // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
  // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
  // message pool is determined by the number of threads (the maximum number of concurrent accesses
  // to the subscription).
  auto state_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointState, 1>>();
  auto setpoint_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointCommand, 1>>();

  this->get_sensor_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCUTILS_LOG_INFO_NAMED(get_name(),
        "Requested deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    };

  sub_sensor_ = this->create_subscription<pendulum_msgs::msg::JointState>(
    "pendulum_sensor", qos_profile_,
    std::bind(&PendulumControllerNode::on_sensor_message,
    this, std::placeholders::_1),
    sensor_subscription_options_,
    state_msg_strategy);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo & event) -> void
    {
      RCUTILS_LOG_INFO_NAMED(get_name(),
        "Offered deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    };
  // Initialize the publisher for the command message.
  command_pub_ = this->create_publisher<pendulum_msgs::msg::JointCommand>(
    "pendulum_command",
    qos_profile_,
    command_publisher_options_);

  setpoint_sub_ = this->create_subscription<pendulum_msgs::msg::JointCommand>(
    "pendulum_setpoint", setpoint_qos_profile_,
    std::bind(&PendulumControllerNode::on_pendulum_setpoint,
    this, std::placeholders::_1),
    rclcpp::SubscriptionOptions(),
    setpoint_msg_strategy);

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
  timer_ =
    this->create_wall_timer(publish_period_,
      std::bind(&PendulumControllerNode::control_timer_callback, this));

  if (check_memory_) {
  #ifdef PENDULUM_CONTROLLER_MEMORYTOOLS_ENABLED
    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_free_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_begin();
  #endif
  }
  show_new_pagefault_count("on_activate", ">=0", ">=0");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  show_new_pagefault_count("on_deactivate", "0", "0");
  if (check_memory_) {
  #ifdef PENDULUM_CONTROLLER_MEMORYTOOLS_ENABLED
    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_end();
    osrf_testing_tools_cpp::memory_tools::expect_no_free_end();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_end();
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_end();
  #endif
  }
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  timer_->cancel();
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
}  // namespace pendulum

void PendulumControllerNode::show_new_pagefault_count(
  const char * logtext,
  const char * allowed_maj,
  const char * allowed_min)
{
  struct rusage usage;

  getrusage(RUSAGE_SELF, &usage);

  RCUTILS_LOG_INFO_NAMED(get_name(),
    "%-30.30s: Pagefaults, Major:%ld (Allowed %s), "
    "Minor:%ld (Allowed %s)", logtext,
    usage.ru_majflt - last_majflt_, allowed_maj,
    usage.ru_minflt - last_minflt_, allowed_min);
  last_majflt_ = usage.ru_majflt;
  last_minflt_ = usage.ru_minflt;
}

}  // namespace pendulum


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumControllerNode)
