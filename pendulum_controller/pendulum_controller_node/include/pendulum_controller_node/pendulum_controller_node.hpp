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

#ifndef PENDULUM_CONTROLLER_NODE__PENDULUM_CONTROLLER_NODE_HPP_
#define PENDULUM_CONTROLLER_NODE__PENDULUM_CONTROLLER_NODE_HPP_

#include <sys/time.h>  // needed for getrusage
#include <sys/resource.h>  // needed for getrusage

#include <pendulum_ex_msgs/msg/controller_stats.hpp>
// #include <pendulum_ex_msgs/msg/rusage.hpp>
// #include <pendulum_ex_msgs/msg/timer_stats.hpp>
// #include <pendulum_ex_msgs/msg/topic_stats.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <memory>
#include <string>
#include <climits>

#ifdef PENDULUM_CONTROLLER_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>
#endif

#include "rcutils/logging_macros.h"
#include "pendulum_ex_msgs/msg/joint_command_ex.hpp"
#include "pendulum_ex_msgs/msg/joint_state_ex.hpp"
#include "pendulum_controller_node/visibility_control.hpp"
#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_tools/timing_analyzer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace pendulum
{

class PendulumControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  COMPOSITION_PUBLIC
  explicit PendulumControllerNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("Controller", options)
  {}

  COMPOSITION_PUBLIC PendulumControllerNode(
    const std::string & node_name,
    std::unique_ptr<PendulumController> controller,
    std::chrono::nanoseconds publish_period,
    const rclcpp::QoS & qos_profile,
    const rclcpp::QoS & setpoint_qos_profile,
    const bool check_memory,
    const rclcpp::NodeOptions & options);

  void on_sensor_message(const pendulum_ex_msgs::msg::JointStateEx::SharedPtr msg);
  void on_pendulum_setpoint(
    const pendulum_ex_msgs::msg::JointCommandEx::SharedPtr msg);

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

  void control_timer_callback();

  /// Get the subscription's settings options.
  rclcpp::SubscriptionOptions & get_sensor_options() {return sensor_subscription_options_;}
  rclcpp::PublisherOptions & get_command_options() {return command_publisher_options_;}
  const pendulum_ex_msgs::msg::ControllerStats & get_controller_stats_message() const;
  void update_sys_usage(bool update_active_page_faults = false);

private:
  std::shared_ptr<rclcpp::Subscription<
      pendulum_ex_msgs::msg::JointStateEx>> sub_sensor_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_ex_msgs::msg::JointCommandEx>> command_pub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_ex_msgs::msg::JointCommandEx>> setpoint_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_ex_msgs::msg::ControllerStats>> logger_pub_;
  std::shared_ptr<rclcpp::Subscription<
      lifecycle_msgs::msg::TransitionEvent>> sub_notification_;

  rclcpp::PublisherOptions command_publisher_options_;
  rclcpp::SubscriptionOptions sensor_subscription_options_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::nanoseconds publish_period_ = std::chrono::nanoseconds(1000000);
  std::unique_ptr<PendulumController> controller_;
  rclcpp::QoS qos_profile_ = rclcpp::QoS(1);
  rclcpp::QoS setpoint_qos_profile_ = rclcpp::QoS(
    rclcpp::KeepLast(10)).transient_local().reliable();
  pendulum_ex_msgs::msg::ControllerStats controller_stats_message_;
  pendulum_ex_msgs::msg::JointStateEx sensor_message_;
  pendulum_ex_msgs::msg::JointCommandEx command_message_;
  rusage sys_usage_;
  uint64_t minor_page_faults_at_active_start_ = 0;
  uint64_t major_page_faults_at_active_start_ = 0;
  bool check_memory_ = false;
  TimingAnalyzer timer_jitter_{std::chrono::nanoseconds(0)};
};

}  // namespace pendulum

#endif  // PENDULUM_CONTROLLER_NODE__PENDULUM_CONTROLLER_NODE_HPP_
