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

#ifndef PENDULUM_MANAGER__PENDULUM_NODE_MANAGER_HPP_
#define PENDULUM_MANAGER__PENDULUM_NODE_MANAGER_HPP_

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

#include "pendulum_manager/lifecycle_service_client.hpp"

namespace pendulum
{

/// \class This class to manage the pendulum managed nodes
class PendulumNodeManager : public rclcpp::Node
{
public:
  /// \brief Constructor of the class
  /// \param[in] node_name name pendulum manager node
  /// \param[in] controller_node_name name of the controlle node
  /// \param[in] driver_node_name name of the driver node
  PendulumNodeManager(
    const std::string & node_name,
    const std::string & controller_node_name,
    const std::string & driver_node_name);

  /// \brief Sends a request to trigger the configure transition in the controller
  void configure_controller() {controller_client_->configure();}
  /// \brief Sends a request to trigger the activate transition in the controller
  void activate_controller() {controller_client_->activate();}
  /// \brief Sends a request to trigger the deactivate transition in the controller
  void deactivate_controller() {controller_client_->deactivate();}
  /// \brief Sends a request to trigger the cleanup transition in the controller
  void cleanup_controller() {controller_client_->cleanup();}
  /// \brief Sends a request to trigger the shutdown transition in the controller
  void shutdown_controller() {controller_client_->shutdown();}
  /// \brief Sends a request to trigger the configure transition in the driver
  void configure_driver() {driver_client_->configure();}
  /// \brief Sends a request to trigger the activate transition in the driver
  void activate_driver() {driver_client_->activate();}
  /// \brief Sends a request to trigger the deactivate transition in the driver
  void deactivate_driver() {driver_client_->deactivate();}
  /// \brief Sends a request to trigger the cleanup transition in the driver
  void cleanup_driver() {driver_client_->cleanup();}
  /// \brief Sends a request to trigger the shutdown transition in the driver
  void shutdown_driver() {driver_client_->shutdown();}

private:
  std::shared_ptr<pendulum::LifecycleServiceClient> controller_client_;
  std::shared_ptr<pendulum::LifecycleServiceClient> driver_client_;
  std::string node_name_;
};

}  // namespace pendulum

#endif  // PENDULUM_MANAGER__PENDULUM_NODE_MANAGER_HPP_
