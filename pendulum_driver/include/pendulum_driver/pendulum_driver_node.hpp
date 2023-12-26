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

#include "pendulum_utils/periodic_task.hpp"
#include "pendulum_utils/RealtimeObject.hpp"
#include "pendulum_utils/statistics_tracker.hpp"
#include "pendulum_utils/process_settings.hpp"
#include "pendulum_driver/pendulum_driver.hpp"
#include "pendulum_driver/visibility_control.hpp"

namespace pendulum_driver
{
/// \class This class implements a node containing a simulated inverted pendulum or
/// the drivers for a real one.
class PendulumDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using RealtimeStatisticsTracker = farbot::RealtimeObject<utils::StatisticsTracker,
      farbot::RealtimeObjectOptions::realtimeMutatable>;
  using ThreadType = farbot::ThreadType;

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

  /// \brief Get the callback group for real-time entities
  [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group() const
  {
    return realtime_cb_group_;
  }

  /// \brief Get the process settings to configure the real-time thread
  utils::ProcessSettings get_proc_settings()
  {
    return proc_settings_;
  }

  /// \brief Start the activity by transitioning the node to active state
  /// \remarks safe to call from real-time thread
  void start();

  /// \brief
  /// \remarks safe to call from real-time thread
  void wait_for_start(bool wait_for_publisher_matched = false);

  /// \brief Run the real-time loop
  /// \remarks safe to call from real-time thread
  void run_realtime_loop();

  /// \brief Real-time loop update
  /// \remarks safe to call from real-time thread
  void update_realtime_loop();

  /// \brief Log pendulum driver state
  void log_driver_state();

  /// \brief Update statistics
  void update_rt_loop_statistics(
    std::chrono::steady_clock::time_point before,
    std::chrono::steady_clock::time_point after);

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

  /// \brief Create wait-set
  void create_wait_set();

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
  std::chrono::microseconds update_period_;
  PendulumDriver driver_;

  std::shared_ptr<rclcpp::Subscription<pendulum2_msgs::msg::JointCommand>> command_sub_;
  std::shared_ptr<rclcpp::Subscription<pendulum2_msgs::msg::JointCommand>> disturbance_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum2_msgs::msg::JointState>> state_pub_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  // rclcpp::TimerBase::SharedPtr update_driver_timer_;
  pendulum2_msgs::msg::JointState state_message_;

  rclcpp::CallbackGroup::SharedPtr realtime_cb_group_;

  /// automatically activate lifecycle nodes
  bool auto_start_node_ = false;

  std::atomic_bool is_active_ = false;

  utils::ProcessSettings proc_settings_;

  std::shared_ptr<rclcpp::StaticWaitSet<0, 0, 1, 0, 0, 0>> wait_set_;

  std::atomic_uint64_t iteration_{0UL};
  std::atomic_uint64_t num_missed_deadlines_{0UL};
  std::atomic_uint64_t num_missed_messages_{0UL};

  RealtimeStatisticsTracker wake_up_latency_tracker_;
  RealtimeStatisticsTracker rt_task_duration_tracker_;

  utils::PeriodInfo period_info_;
};
}  // namespace pendulum_driver

#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_NODE_HPP_
