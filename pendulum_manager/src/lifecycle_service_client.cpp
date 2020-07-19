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

#include "pendulum_manager/lifecycle_service_client.hpp"
#include <memory>

namespace pendulum
{

unsigned int
LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  if (!client_get_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(lifecycle_node_),
      "Service %s is not available.",
      client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  // We send the service request for asking the current
  // state of the lc_talker node.
  auto future_result = client_get_state_->async_send_request(request);

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      rclcpp::get_logger(lifecycle_node_),
      "Server time out while getting current state for node %s", lifecycle_node_);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  // We have an succesful answer. So let's print the current state.
  if (future_result.get()) {
    RCLCPP_INFO(
      rclcpp::get_logger(lifecycle_node_), "Current state %s.",
      future_result.get()->current_state.label.c_str());
    return future_result.get()->current_state.id;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(lifecycle_node_),
      "Failed to get current state for node %s", lifecycle_node_);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

bool
LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  if (!client_change_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(lifecycle_node_),
      "Service %s is not available.",
      client_change_state_->get_service_name());
    return false;
  }

  // We send the request with the transition we want to invoke.
  auto future_result = client_change_state_->async_send_request(request);

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      rclcpp::get_logger(lifecycle_node_),
      "Server time out while getting current state for node %s", lifecycle_node_);
    return false;
  }

  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(
      rclcpp::get_logger(lifecycle_node_),
      "Transition %d successfully triggered.", static_cast<int>(transition));
    return true;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger(lifecycle_node_),
      "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return false;
  }
}

void LifecycleServiceClient::configure()
{
  if (!rclcpp::ok()) {
    return;
  }
  if (!change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
    return;
  }
  if (!get_state()) {
    return;
  }
}

void LifecycleServiceClient::activate()
{
  if (!rclcpp::ok()) {
    return;
  }
  if (!change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
    return;
  }
  if (!get_state()) {
    return;
  }
}

void LifecycleServiceClient::deactivate()
{
  if (!rclcpp::ok()) {
    return;
  }
  if (!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
    return;
  }
  if (!get_state()) {
    return;
  }
}

void LifecycleServiceClient::cleanup()
{
  if (!rclcpp::ok()) {
    return;
  }
  if (!change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
    return;
  }
  if (!get_state()) {
    return;
  }
}

void LifecycleServiceClient::shutdown()
{
  if (!rclcpp::ok()) {
    return;
  }
  if (!change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)) {
    return;
  }
  if (!get_state()) {
    return;
  }
}

}  // namespace pendulum
