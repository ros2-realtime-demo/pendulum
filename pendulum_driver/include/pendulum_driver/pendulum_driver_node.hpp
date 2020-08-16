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
  PENDULUM_DRIVER_PUBLIC PendulumDriverNode(
    const std::string & node_name,
    rclcpp::NodeOptions options = rclcpp::NodeOptions());

  /// \brief Explicit constructor
  /// \param[in] node_name Name of this node
  /// \param[in] sensor_topic_name Name of the sensor state topic
  /// \param[in] command_topic_name Name of the command topic
  /// \param[in] disturbance_topic_name Name of the disturbance topic
  /// \param[in] status_publish_period Period of the sensor state topic publishing
  /// \param[in] driver_cfg Configuration class for the pendulum driver
  PENDULUM_DRIVER_PUBLIC PendulumDriverNode(
    const std::string & node_name,
    const std::string & sensor_topic_name,
    const std::string & command_topic_name,
    const std::string & disturbance_topic_name,
    const std::string & cart_base_joint_name,
    const std::string & pole_joint_name,
    std::chrono::microseconds status_publish_period,
    bool enable_topic_stats,
    const std::string & topic_stats_topic_name,
    std::chrono::milliseconds topic_stats_publish_period,
    std::chrono::milliseconds deadline_duration,
    const PendulumDriver::Config & driver_cfg);

  /// \brief Initialize pendulum driver
  void init();

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
  const std::string sensor_topic_name_;
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

  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> command_sub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> disturbance_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::JointState>> state_pub_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr update_driver_timer_;
  sensor_msgs::msg::JointState state_message_;
};
}  // namespace pendulum_driver
}  // namespace pendulum

#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_NODE_HPP_
