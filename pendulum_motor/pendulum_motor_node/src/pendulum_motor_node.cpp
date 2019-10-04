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

#include "pendulum_motor_node/pendulum_motor_node.hpp"
#include "rttest/utils.h"

namespace pendulum
{

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

PendulumMotorNode::PendulumMotorNode(
  const std::string & node_name,
  std::unique_ptr<PendulumMotor> motor,
  std::chrono::nanoseconds publish_period,
  const rclcpp::QoS & qos_profile,
  const bool check_memory = false,
  const rclcpp::NodeOptions & options =
  rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  publish_period_(publish_period),
  motor_(std::move(motor)),
  qos_profile_(qos_profile),
  check_memory_(check_memory),
  timer_jitter_(publish_period)
{
  if (check_memory_) {
  #ifdef PENDULUM_MOTOR_MEMORYTOOLS_ENABLED
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

void PendulumMotorNode::on_command_received(
  const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg)
{
  motor_stats_message_.sensor_stats.msg_count++;
  motor_->update_command_data(*msg);
}

void PendulumMotorNode::sensor_timer_callback()
{
  motor_->update_sensor_data(sensor_message_);
  sensor_pub_->publish(sensor_message_);
  motor_stats_message_.command_stats.msg_count++;
  motor_stats_message_.timer_stats.timer_count++;
  timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  motor_stats_message_.timer_stats.stamp.sec = curtime.tv_sec;
  motor_stats_message_.timer_stats.stamp.nanosec = curtime.tv_nsec;

  timer_jitter_.update();
  motor_stats_message_.timer_stats.jitter_mean_nsec = timer_jitter_.get_mean();
  motor_stats_message_.timer_stats.jitter_min_nsec = timer_jitter_.get_min();
  motor_stats_message_.timer_stats.jitter_max_nsec = timer_jitter_.get_max();
  motor_stats_message_.timer_stats.jitter_standard_dev_nsec = timer_jitter_.get_std();
}

void PendulumMotorNode::update_motor_callback()
{
  motor_->update();
}

const pendulum_msgs_v2::msg::MotorStats &
PendulumMotorNode::get_motor_stats_message() const
{
  return motor_stats_message_;
}

// TODO(carlossvg): this function may be duplicated, move it to a tools package
void PendulumMotorNode::update_sys_usage(bool update_active_page_faults)
{
  const auto ret = getrusage(RUSAGE_SELF, &sys_usage_);
  if (ret == 0) {
    motor_stats_message_.rusage_stats.max_resident_set_size = sys_usage_.ru_maxrss;
    motor_stats_message_.rusage_stats.total_minor_pagefaults = sys_usage_.ru_minflt;
    motor_stats_message_.rusage_stats.total_major_pagefaults = sys_usage_.ru_majflt;
    motor_stats_message_.rusage_stats.voluntary_context_switches = sys_usage_.ru_nvcsw;
    motor_stats_message_.rusage_stats.involuntary_context_switches = sys_usage_.ru_nivcsw;
    if (update_active_page_faults) {
      minor_page_faults_at_active_start_ = sys_usage_.ru_minflt;
      major_page_faults_at_active_start_ = sys_usage_.ru_majflt;
    }
    if (this->get_current_state().label() == "active") {
      motor_stats_message_.rusage_stats.minor_pagefaults_active_node =
        sys_usage_.ru_minflt - minor_page_faults_at_active_start_;
      motor_stats_message_.rusage_stats.major_pagefaults_active_node =
        sys_usage_.ru_majflt - major_page_faults_at_active_start_;
    }
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

  // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
  // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
  // message pool is determined by the number of threads (the maximum number of concurrent accesses
  // to the subscription).
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs_v2::msg::PendulumCommand, 1>>();

  this->get_sensor_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      this->motor_stats_message_.sensor_stats.deadline_misses_count++;
    };
  sensor_pub_ = this->create_publisher<pendulum_msgs_v2::msg::PendulumState>(
    "pendulum_sensor", qos_profile_, sensor_publisher_options_);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      this->motor_stats_message_.command_stats.deadline_misses_count++;
    };
  command_sub_ = this->create_subscription<pendulum_msgs_v2::msg::PendulumCommand>(
    "pendulum_command", qos_profile_,
    std::bind(&PendulumMotorNode::on_command_received,
    this, std::placeholders::_1),
    command_subscription_options_,
    command_msg_strategy);

  sensor_timer_ =
    this->create_wall_timer(publish_period_,
      std::bind(&PendulumMotorNode::sensor_timer_callback, this));
  // cancel immediately to prevent triggering it in this state
  sensor_timer_->cancel();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  sensor_pub_->on_activate();
  sensor_timer_->reset();

  update_sys_usage(true);
  if (check_memory_) {
  #ifdef PENDULUM_MOTOR_MEMORYTOOLS_ENABLED
    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_free_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_begin();
  #endif
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (check_memory_) {
  #ifdef PENDULUM_MOTOR_MEMORYTOOLS_ENABLED
    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_end();
    osrf_testing_tools_cpp::memory_tools::expect_no_free_end();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_end();
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_end();
  #endif
  }

  update_sys_usage(false);
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  sensor_timer_->cancel();
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

}  // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumMotorNode)
