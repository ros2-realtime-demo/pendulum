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
/// \brief This file provides a ROS 2 interface to implement the inverted pendulum driver.

#ifndef PENDULUM_DRIVER__PENDULUM_DRIVER_NODE_HPP_
#define PENDULUM_DRIVER__PENDULUM_DRIVER_NODE_HPP_

#include <sys/time.h>  // needed for getrusage
#include <sys/resource.h>  // needed for getrusage

#include <pendulum_msgs_v2/msg/pendulum_stats.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <memory>
#include <string>

#ifdef PENDULUM_DRIVER_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>
#endif

#include "rcutils/logging_macros.h"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "pendulum_msgs_v2/msg/pendulum_command.hpp"
#include "pendulum_msgs_v2/msg/pendulum_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pendulum_tools/resource_usage.hpp"
#include "pendulum_tools/jitter_tracker.hpp"
#include "pendulum_driver/pendulum_driver_interface.hpp"
#include "pendulum_driver/visibility_control.hpp"

namespace pendulum
{

struct PendulumDriverOptions
{
  std::string node_name = "pendulum_driver";
  std::chrono::nanoseconds status_publish_period = std::chrono::nanoseconds(1E6);
  rclcpp::QoS status_qos_profile = rclcpp::QoS(1);
  bool enable_statistics = false;
  std::chrono::nanoseconds statistics_publish_period = std::chrono::nanoseconds(1E9);
  bool enable_check_memory = false;
};

/// \class This class implements a nodstd::unique_ptr<PendulumDriverInterface> driver_interface_;e containing a the a simulated inverted pendulum or
///        the drivers for a real one.
class PendulumDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \brief Default constructor, needed for node composition
  /// \param[in] options Node options for rclcpp internals
  COMPOSITION_PUBLIC
  explicit PendulumDriverNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("pendulum_driver", options)
  {}

  /// \brief Main constructor with parameters
  /// \param[in] node_name Name of the node for rclcpp internals
  /// \param[in] driver_interface Pointer to the driver implementation
  /// \param[in] publish_period Period for the driver node status publishing
  /// \param[in] status_qos_profile QoS profile for comamnd and status topics
  /// \param[in] enable_check_memory Flag to enable memory allocation checking
  /// \param[in] options Node options for rclcpp internals
  /// \throw std::runtime_error If memory checking is enabled but not working or not installed.
  COMPOSITION_PUBLIC PendulumDriverNode(
    std::unique_ptr<PendulumDriverInterface> driver_interface,
    PendulumDriverOptions driver_options,
    const rclcpp::NodeOptions & options);


  /// \brief Get the command subscription's settings options.
  /// \return  subscription's settings options
  rclcpp::SubscriptionOptions & get_command_options() {return command_subscription_options_;}

  /// \brief Get the state publisher's settings options.
  /// \return  publisher's settings options
  rclcpp::PublisherOptions & get_state_options() {return sensor_publisher_options_;}

  /// \brief Get the driver statistics message.
  /// \return  last driver statistics message
  const pendulum_msgs_v2::msg::PendulumStats & get_stats_message() const;

  // /// \brief Update system usage statistics
  // /// \param[in] update_active_page_faults update paga faults only in active state
  // void update_sys_usage(bool update_active_page_faults = false);

private:
  /// \brief pendulum command topic message callback
  /// \param[in] msg pendulum command message
  void on_command_received(const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg);

  /// \brief pendulum disturbance topic message callback
  /// \param[in] msg pendulum disturbance message
  void on_disturbance_received(const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg);

  /// \brief pendulum state publish timer callback
  void state_timer_callback();

  /// \brief pendulum internal status update timer callback
  void update_driver_callback();

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
  std::unique_ptr<PendulumDriverInterface> driver_interface_;
  PendulumDriverOptions driver_options_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::JointState>> sensor_pub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> command_sub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> disturbance_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_msgs_v2::msg::PendulumStats>> statistics_pub_;

  rclcpp::SubscriptionOptions command_subscription_options_;
  rclcpp::SubscriptionOptions disturbance_subscription_options_;
  rclcpp::PublisherOptions sensor_publisher_options_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  rclcpp::TimerBase::SharedPtr update_driver_timer_;

  pendulum_msgs_v2::msg::PendulumStats statistics_message_;
  sensor_msgs::msg::JointState state_message_;
  pendulum_msgs_v2::msg::PendulumCommand command_message_;
  pendulum_msgs_v2::msg::PendulumCommand disturbance_message_;

  JitterTracker timer_jitter_{std::chrono::nanoseconds(0)};
  ResourceUsage resource_usage_;
};

}  // namespace pendulum

#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_NODE_HPP_
