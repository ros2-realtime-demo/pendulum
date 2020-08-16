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

#ifndef PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_NODE_HPP_
#define PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_NODE_HPP_

#include <string>
#include <climits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pendulum_msgs_v2/msg/pendulum_command.hpp"
#include "pendulum_msgs_v2/msg/pendulum_state.hpp"
#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_controller/visibility_control.hpp"

namespace pendulum
{
namespace pendulum_controller
{
/// \class This class implements a node containing a controller for the inverted pendulum.
class PendulumControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \brief Default constructor, needed for node composition
  /// \param[in] options Node options for rclcpp internals
  PENDULUM_CONTROLLER_PUBLIC
  explicit PendulumControllerNode(const rclcpp::NodeOptions & options);

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] options Node options for rclcpp internals
  PENDULUM_CONTROLLER_PUBLIC PendulumControllerNode(
    const std::string & node_name,
    rclcpp::NodeOptions options = rclcpp::NodeOptions());
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

  const std::string sensor_topic_name_;
  const std::string command_topic_name_;
  const std::string setpoint_topic_name_;
  std::chrono::microseconds command_publish_period_;
  bool enable_topic_stats_;
  const std::string topic_stats_topic_name_;
  std::chrono::milliseconds topic_stats_publish_period_;
  std::chrono::milliseconds deadline_duration_;

  PendulumController controller_;

  std::shared_ptr<rclcpp::Subscription<
      sensor_msgs::msg::JointState>> state_sub_;
  std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs_v2::msg::PendulumCommand>> setpoint_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_msgs_v2::msg::PendulumCommand>> command_pub_;

  rclcpp::TimerBase::SharedPtr command_timer_;
  pendulum_msgs_v2::msg::PendulumCommand command_message_;
};
}  // namespace pendulum_controller
}  // namespace pendulum

#endif  // PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_NODE_HPP_
