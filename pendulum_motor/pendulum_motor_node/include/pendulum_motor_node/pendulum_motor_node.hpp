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

#ifndef PENDULUM_MOTOR_NODE__PENDULUM_MOTOR_NODE_HPP_
#define PENDULUM_MOTOR_NODE__PENDULUM_MOTOR_NODE_HPP_

#include <sys/time.h>  // needed for getrusage
#include <sys/resource.h>  // needed for getrusage

#ifdef PENDULUM_MOTOR_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>
#endif

#include <pendulum_msgs/msg/rttest_results.hpp>

#include <memory>
#include <string>
#include <cmath>

#include "rcutils/logging_macros.h"

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

#include "pendulum_motor_driver/pendulum_motor_driver.hpp"
#include "pendulum_motor_node/visibility_control.hpp"

namespace pendulum
{

class PendulumMotorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  COMPOSITION_PUBLIC
  explicit PendulumMotorNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("PendulumMotor", options),
    qos_profile_(rclcpp::QoS(1))
  {}
  COMPOSITION_PUBLIC PendulumMotorNode(
    const std::string & node_name,
    std::unique_ptr<PendulumMotor> motor,
    std::chrono::nanoseconds publish_period,
    const rclcpp::QoS & qos_profile,
    const bool check_memory,
    const rclcpp::NodeOptions & options);
  void on_command_received(const pendulum_msgs::msg::JointCommand::SharedPtr msg);
  void sensor_timer_callback();
  void update_motor_callback();

  /// Get the subscription's settings options.
  rclcpp::SubscriptionOptions & get_command_options() {return command_subscription_options_;}
  rclcpp::PublisherOptions & get_sensor_options() {return sensor_publisher_options_;}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

  void show_new_pagefault_count(
    const char * logtext,
    const char * allowed_maj,
    const char * allowed_min);

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_msgs::msg::JointState>> sensor_pub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs::msg::JointCommand>> command_sub_;

  rclcpp::SubscriptionOptions command_subscription_options_;
  rclcpp::PublisherOptions sensor_publisher_options_;

  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr update_motor_timer_;
  std::chrono::nanoseconds publish_period_ = std::chrono::nanoseconds(1000000);
  pendulum_msgs::msg::JointState sensor_message_;
  std::unique_ptr<PendulumMotor> motor_;
  rclcpp::QoS qos_profile_;
  int last_majflt_ = 0;
  int last_minflt_ = 0;
  bool check_memory_ = false;
};

}  // namespace pendulum

#endif  // PENDULUM_MOTOR_NODE__PENDULUM_MOTOR_NODE_HPP_
