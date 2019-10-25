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

#ifndef PENDULUM_MANAGER__LIFECYCLE_SERVICE_CLIENT_HPP_
#define PENDULUM_MANAGER__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcutils/logging_macros.h"

namespace pendulum
{

/// Helper function to wait for response
template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleServiceClient
{
public:
  LifecycleServiceClient(
    const std::string & lifecycle_node,
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state,
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state)
  : lifecycle_node_(lifecycle_node),
    client_get_state_(client_get_state),
    client_change_state_(client_change_state)
  {
  }

  /// \brief Sends a request to trigger the configure transition
  void configure();
  /// \brief Sends a request to trigger the activate transition
  void activate();
  /// \brief Sends a request to trigger the deactivate transition
  void deactivate();
  /// \brief Sends a request to trigger the cleanup transition
  void cleanup();
  /// \brief Sends a request to trigger the shutdown transition
  void shutdown();

private:
  /// \brief Requests the current state of the node
  unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));

  /// \brief Invokes a transition of the lifecycle node
  bool change_state(
    std::uint8_t transition,
    std::chrono::seconds time_out = std::chrono::seconds(3));

  std::string lifecycle_node_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr_;
};

}  // namespace pendulum

#endif  // PENDULUM_MANAGER__LIFECYCLE_SERVICE_CLIENT_HPP_
