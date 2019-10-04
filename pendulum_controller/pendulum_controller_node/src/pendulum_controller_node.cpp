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
  check_memory_(check_memory),
  timer_jitter_(publish_period)
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
  const pendulum_msgs_v2::msg::PendulumState::SharedPtr msg)
{
  controller_stats_message_.sensor_stats.msg_count++;
  controller_->update_sensor_data(*msg);
}

void PendulumControllerNode::on_pendulum_setpoint(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  controller_stats_message_.setpoint_stats.msg_count++;
  controller_->update_setpoint_data(*msg);
}

void PendulumControllerNode::control_timer_callback()
{
  controller_->update_command_data(command_message_);
  command_pub_->publish(command_message_);
  controller_stats_message_.command_stats.msg_count++;
  controller_stats_message_.timer_stats.timer_count++;
  timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  controller_stats_message_.timer_stats.stamp.sec = curtime.tv_sec;
  controller_stats_message_.timer_stats.stamp.nanosec = curtime.tv_nsec;
  timer_jitter_.update();
  controller_stats_message_.timer_stats.jitter_mean_nsec = timer_jitter_.get_mean();
  controller_stats_message_.timer_stats.jitter_min_nsec = timer_jitter_.get_min();
  controller_stats_message_.timer_stats.jitter_max_nsec = timer_jitter_.get_max();
  controller_stats_message_.timer_stats.jitter_standard_dev_nsec = timer_jitter_.get_std();
}

const pendulum_msgs_v2::msg::ControllerStats &
PendulumControllerNode::get_controller_stats_message() const
{
  return controller_stats_message_;
}

// TODO(carlossvg): this function may be duplicated, move it to a tools package
void PendulumControllerNode::update_sys_usage(bool update_active_page_faults)
{
  const auto ret = getrusage(RUSAGE_SELF, &sys_usage_);
  if (ret == 0) {
    controller_stats_message_.rusage_stats.max_resident_set_size = sys_usage_.ru_maxrss;
    controller_stats_message_.rusage_stats.total_minor_pagefaults = sys_usage_.ru_minflt;
    controller_stats_message_.rusage_stats.total_major_pagefaults = sys_usage_.ru_majflt;
    controller_stats_message_.rusage_stats.voluntary_context_switches = sys_usage_.ru_nvcsw;
    controller_stats_message_.rusage_stats.involuntary_context_switches = sys_usage_.ru_nivcsw;
    if (update_active_page_faults) {
      minor_page_faults_at_active_start_ = sys_usage_.ru_minflt;
      major_page_faults_at_active_start_ = sys_usage_.ru_majflt;
    }
    if (this->get_current_state().label() == "active") {
      controller_stats_message_.rusage_stats.minor_pagefaults_active_node =
        sys_usage_.ru_minflt - minor_page_faults_at_active_start_;
      controller_stats_message_.rusage_stats.major_pagefaults_active_node =
        sys_usage_.ru_majflt - major_page_faults_at_active_start_;
    }
  }
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
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumState, 1>>();
  auto setpoint_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  this->get_sensor_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      controller_stats_message_.sensor_stats.deadline_misses_count++;
    };

  sub_sensor_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumState>(
    "pendulum_sensor", qos_profile_,
    std::bind(&PendulumControllerNode::on_sensor_message,
    this, std::placeholders::_1),
    sensor_subscription_options_,
    state_msg_strategy);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      controller_stats_message_.command_stats.deadline_misses_count++;
    };
  // Initialize the publisher for the command message.
  command_pub_ = this->create_publisher<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_command",
    qos_profile_,
    command_publisher_options_);

  setpoint_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_setpoint", setpoint_qos_profile_,
    std::bind(&PendulumControllerNode::on_pendulum_setpoint,
    this, std::placeholders::_1),
    rclcpp::SubscriptionOptions(),
    setpoint_msg_strategy);

  timer_ =
    this->create_wall_timer(publish_period_,
      std::bind(&PendulumControllerNode::control_timer_callback, this));
  timer_->cancel();

  // Initialize the logger publisher.
  logger_pub_ = this->create_publisher<pendulum_msgs_v2::msg::ControllerStats>(
    "pendulum_statistics", 1);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  command_pub_->on_activate();
  logger_pub_->on_activate();
  timer_->reset();

  if (check_memory_) {
  #ifdef PENDULUM_CONTROLLER_MEMORYTOOLS_ENABLED
    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_free_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_begin();
  #endif
  }

  update_sys_usage(true);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  update_sys_usage(false);
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

}  // namespace pendulum


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumControllerNode)
