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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pendulum_msgs_v2/msg/pendulum_command.hpp"
#include "pendulum_msgs_v2/msg/pendulum_state.hpp"
#include "pendulum_driver/pendulum_driver_interface.hpp"
#include "pendulum_driver/visibility_control.hpp"

namespace pendulum
{

struct PendulumDriverOptions
{
  std::string node_name = "pendulum_driver";
  std::chrono::microseconds status_publish_period = std::chrono::microseconds(1000);
  rclcpp::QoS status_qos_profile = rclcpp::QoS(10);
};

/// \class This class implements a node containing a the a simulated inverted pendulum or
/// the drivers for a real one.
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
  /// \param[in] driver_interface Pointer to the driver implementation
  /// \param[in] driver_options Options to configure the object
  /// \param[in] options Node options for rclcpp internals
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

private:
  /// \brief pendulum command topic message callback
  /// \param[in] msg pendulum command message
  void on_command_received(const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg);

  /// \brief pendulum disturbance topic message callback
  /// \param[in] msg pendulum disturbance message
  void on_disturbance_received(const pendulum_msgs_v2::msg::PendulumCommand::SharedPtr msg);

  /// \brief pendulum state publish timer callback
  void state_timer_callback();

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

  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> command_sub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> disturbance_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::JointState>> state_pub_;

  rclcpp::SubscriptionOptions command_subscription_options_;
  rclcpp::SubscriptionOptions disturbance_subscription_options_;
  rclcpp::PublisherOptions sensor_publisher_options_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr update_driver_timer_;

  sensor_msgs::msg::JointState state_message_;
  pendulum_msgs_v2::msg::PendulumCommand command_message_;
  pendulum_msgs_v2::msg::PendulumCommand disturbance_message_;
};
}  // namespace pendulum

#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_NODE_HPP_
