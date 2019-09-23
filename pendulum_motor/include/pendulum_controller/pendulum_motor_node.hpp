// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef PENDULUM__MOTOR_NODE_HPP_
#define PENDULUM__MOTOR_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"
#include <pendulum_msgs/msg/rttest_results.hpp>

#include "pendulum_controller/visibility_control.h"

namespace pendulum
{

class MotorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit MotorNode(const rclcpp::NodeOptions & options);

    void on_command_received(const pendulum_msgs::msg::JointCommand::SharedPtr msg);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state);


private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum_msgs::msg::JointState>> sensor_pub;
    std::shared_ptr<rclcpp::Subscription<pendulum_msgs::msg::JointCommand>> command_sub;
};

}  // namespace pendulum

#endif  // PENDULUM__MOTOR_NODE_HPP_
