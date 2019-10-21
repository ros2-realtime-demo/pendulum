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

/// \file
/// \brief This file provides a ROS 2 interface to implement the inverted pendulum controller.

#ifndef PENDULUM_CONTROLLER_NODE__PENDULUM_CONTROLLER_NODE_HPP_
#define PENDULUM_CONTROLLER_NODE__PENDULUM_CONTROLLER_NODE_HPP_

#include <sys/time.h>  // needed for getrusage
#include <sys/resource.h>  // needed for getrusage

#include <pendulum_msgs_v2/msg/controller_stats.hpp>
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
#include "pendulum_msgs_v2/msg/pendulum_command.hpp"
#include "pendulum_msgs_v2/msg/pendulum_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pendulum_controller_node/visibility_control.hpp"
#include "pendulum_controller_node/pendulum_controller.hpp"
#include "pendulum_tools/timing_analyzer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace pendulum
{

/// \class This class implements a node containing a controller for the inverted pendulum.
class PendulumControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \brief Default constructor, needed for node composition
  /// \param[in] options Node options for rclcpp internals
  COMPOSITION_PUBLIC
  explicit PendulumControllerNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("pendulum_controller", options)
  {}

  /// \brief Main constructor with parameters
  /// \param[in] node_name Name of the node for rclcpp internals
  /// \param[in] controller Pointer to the controller implementation
  /// \param[in] publish_period Period for the controller command publishing
  /// \param[in] qos_profile QoS profile for comamnd and status topics
  /// \param[in] setpoint_qos_profile QoS profile for the setpoint topic
  /// \param[in] check_memory Flag to enable memory allocation checking
  /// \param[in] options Node options for rclcpp internals
  /// \throw std::runtime_error If memory checking is enabled but not working or not installed.
  COMPOSITION_PUBLIC PendulumControllerNode(
    const std::string & node_name,
    std::unique_ptr<PendulumController> controller,
    std::chrono::nanoseconds publish_period,
    const rclcpp::QoS & qos_profile,
    const rclcpp::QoS & setpoint_qos_profile,
    const bool check_memory,
    const rclcpp::NodeOptions & options);

  /// \brief Get the sensor subscription's settings options.
  /// \return  subscription's settings options
  rclcpp::SubscriptionOptions & get_state_options() {return sensor_subscription_options_;}

  /// \brief Get the command publisher's settings options.
  /// \return  publisher's settings options
  rclcpp::PublisherOptions & get_command_options() {return command_publisher_options_;}

  /// \brief Get the controller statistics message.
  /// \return  last controller statistics message
  const pendulum_msgs_v2::msg::ControllerStats & get_controller_stats_message() const;

  /// \brief Update system usage statistics
  /// \param[in] update_active_page_faults update paga faults only in active state
  void update_sys_usage(bool update_active_page_faults = false);

private:
  /// \brief pendulum state topic message callback
  /// \param[in] msg pendulum state message
  void on_sensor_message(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// \brief pendulum setpoint topic message callback
  /// \param[in] msg pendulum setpoint message
  void on_pendulum_setpoint(
    const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg);

  /// \brief controller command publishtimer callback
  void control_timer_callback();

  /// \brief Transition callback for state configuring
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  /// \brief Transition callback for state activating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  /// \brief Transition callback for state deactivating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  /// \brief Transition callback for state cleaningup
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  /// \brief Transition callback for state shutting down
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

private:
  std::shared_ptr<rclcpp::Subscription<
      sensor_msgs::msg::JointState>> state_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_msgs_v2::msg::PendulumCommand>> command_pub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> setpoint_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_msgs_v2::msg::ControllerStats>> logger_pub_;
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
  pendulum_msgs_v2::msg::ControllerStats controller_stats_message_;
  sensor_msgs::msg::JointState state_message_;
  pendulum_msgs_v2::msg::PendulumCommand command_message_;
  rusage sys_usage_;
  uint64_t minor_page_faults_at_active_start_ = 0;
  uint64_t major_page_faults_at_active_start_ = 0;
  bool check_memory_ = false;
  TimingAnalyzer timer_jitter_{std::chrono::nanoseconds(0)};
};
}  // namespace pendulum

#endif  // PENDULUM_CONTROLLER_NODE__PENDULUM_CONTROLLER_NODE_HPP_
