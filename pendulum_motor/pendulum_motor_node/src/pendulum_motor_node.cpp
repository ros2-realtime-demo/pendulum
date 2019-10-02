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
  check_memory_(check_memory)
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
  const pendulum_ex_msgs::msg::JointCommandEx::SharedPtr msg)
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

  // TODO(carlossvg): update other fields
  // builtin_interfaces/Time stamp
  // uint64 timer_count
  // uint64 cur_latency
  // float64 mean_latency
  // uint64 min_latency
  // uint64 max_latency
}

void PendulumMotorNode::update_motor_callback()
{
  motor_->update();
}

const pendulum_ex_msgs::msg::MotorStats &
PendulumMotorNode::get_motor_stats_message() const
{
  return motor_stats_message_;
}

// TODO(carlossvg): this function may be duplicated, move it to a tools package
void PendulumMotorNode::update_sys_usage()
{
  const auto ret = getrusage(RUSAGE_SELF, &sys_usage_);
  if (ret == 0) {
    motor_stats_message_.rusage_stats.ru_maxrss = sys_usage_.ru_maxrss;
    motor_stats_message_.rusage_stats.ru_ixrss = sys_usage_.ru_ixrss;
    motor_stats_message_.rusage_stats.ru_idrss = sys_usage_.ru_idrss;
    motor_stats_message_.rusage_stats.ru_isrss = sys_usage_.ru_isrss;
    motor_stats_message_.rusage_stats.ru_minflt = sys_usage_.ru_minflt;
    motor_stats_message_.rusage_stats.ru_majflt = sys_usage_.ru_majflt;
    motor_stats_message_.rusage_stats.ru_nswap = sys_usage_.ru_nswap;
    motor_stats_message_.rusage_stats.ru_inblock = sys_usage_.ru_inblock;
    motor_stats_message_.rusage_stats.ru_oublock = sys_usage_.ru_oublock;
    motor_stats_message_.rusage_stats.ru_msgsnd = sys_usage_.ru_msgsnd;
    motor_stats_message_.rusage_stats.ru_msgrcv = sys_usage_.ru_msgrcv;
    motor_stats_message_.rusage_stats.ru_nsignals = sys_usage_.ru_nsignals;
    motor_stats_message_.rusage_stats.ru_nvcsw = sys_usage_.ru_nvcsw;
    motor_stats_message_.rusage_stats.ru_nivcsw = sys_usage_.ru_nivcsw;
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
    std::make_shared<MessagePoolMemoryStrategy<pendulum_ex_msgs::msg::JointCommandEx, 1>>();

  this->get_sensor_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      this->motor_stats_message_.sensor_stats.deadline_misses_count++;
    };
  sensor_pub_ = this->create_publisher<pendulum_ex_msgs::msg::JointStateEx>(
    "pendulum_sensor", qos_profile_, sensor_publisher_options_);

  this->get_command_options().event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      this->motor_stats_message_.command_stats.deadline_misses_count++;
    };
  command_sub_ = this->create_subscription<pendulum_ex_msgs::msg::JointCommandEx>(
    "pendulum_command", qos_profile_,
    std::bind(&PendulumMotorNode::on_command_received,
    this, std::placeholders::_1),
    command_subscription_options_,
    command_msg_strategy);

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumMotorNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  sensor_pub_->on_activate();
  sensor_timer_ =
    this->create_wall_timer(publish_period_,
      std::bind(&PendulumMotorNode::sensor_timer_callback, this));

  show_new_pagefault_count("on_activate", ">=0", ">=0");
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

  show_new_pagefault_count("on_deactivate", "0", "0");
  RCUTILS_LOG_INFO_NAMED(get_name(),
    "Sensor requested deadline missed total:  %lu", sensor_missed_deadlines_count_);
  RCUTILS_LOG_INFO_NAMED(get_name(),
    "Command offered deadline missed total:  %lu", command_missed_deadlines_count_);

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

void PendulumMotorNode::show_new_pagefault_count(
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
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::PendulumMotorNode)
