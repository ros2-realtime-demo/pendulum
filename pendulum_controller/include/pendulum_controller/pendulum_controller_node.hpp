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
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

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
  PENDULUM_CONTROLLER_PUBLIC explicit PendulumControllerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// \brief Create teleoperation subscription
  void create_teleoperation_subscription();

  /// \brief Create state subscription
  void create_state_subscription();

  /// \brief Create command publisher
  void create_command_publisher();

  /// \brief Log pendulum controller state
  void log_controller_state();

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
  const std::string teleop_topic_name_;
  bool enable_topic_stats_;
  const std::string topic_stats_topic_name_;
  std::chrono::milliseconds topic_stats_publish_period_;
  std::chrono::milliseconds deadline_duration_;

  PendulumController controller_;

  std::shared_ptr<rclcpp::Subscription<pendulum2_msgs::msg::JointState>> state_sub_;
  std::shared_ptr<rclcpp::Subscription<pendulum2_msgs::msg::PendulumTeleop>> teleop_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum2_msgs::msg::JointCommand>> command_pub_;
  pendulum2_msgs::msg::JointCommand command_message_;

  uint32_t num_missed_deadlines_pub_;
  uint32_t num_missed_deadlines_sub_;
};
}  // namespace pendulum_controller
}  // namespace pendulum

#endif  // PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_NODE_HPP_
