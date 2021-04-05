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

#include "pendulum_driver/pendulum_driver.hpp"
#include "pendulum_driver/visibility_control.hpp"

namespace pendulum
{
namespace pendulum_driver
{
/// \class This class implements a node containing a simulated inverted pendulum or
/// the drivers for a real one.
class PendulumDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \brief Default constructor, needed for node composition
  /// \param[in] options Node options for rclcpp internals
  PENDULUM_DRIVER_PUBLIC
  explicit PendulumDriverNode(const rclcpp::NodeOptions & options);

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] options Node options for rclcpp internals
  PENDULUM_DRIVER_PUBLIC explicit PendulumDriverNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// \brief Initialize state message
  void init_state_message();

  /// \brief Create command subscription
  void create_command_subscription();

  /// \brief Create disturbance subscription
  void create_disturbance_subscription();

  /// \brief Create state publisher
  void create_state_publisher();

  /// \brief Create timer callback
  void create_state_timer_callback();

  /// \brief Log pendulum driver state
  void log_driver_state();

  /// \brief Transition callback for state configuring
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state activating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state deactivating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state cleaningup
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  /// \brief Transition callback for state shutting down
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  const std::string state_topic_name_;
  const std::string command_topic_name_;
  const std::string disturbance_topic_name_;
  const std::string cart_base_joint_name_;
  const std::string pole_joint_name_;
  std::chrono::microseconds state_publish_period_;
  bool enable_topic_stats_;
  const std::string topic_stats_topic_name_;
  std::chrono::milliseconds topic_stats_publish_period_;
  std::chrono::milliseconds deadline_duration_;
  PendulumDriver driver_;

  std::shared_ptr<rclcpp::Subscription<pendulum2_msgs::msg::JointCommand>> command_sub_;
  std::shared_ptr<rclcpp::Subscription<pendulum2_msgs::msg::JointCommand>> disturbance_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum2_msgs::msg::JointState>> state_pub_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr update_driver_timer_;
  pendulum2_msgs::msg::JointState state_message_;

  uint32_t num_missed_deadlines_pub_;
  uint32_t num_missed_deadlines_sub_;
};
}  // namespace pendulum_driver
}  // namespace pendulum

#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_NODE_HPP_
