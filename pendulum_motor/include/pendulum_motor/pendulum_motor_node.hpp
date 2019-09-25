// Copyright 2019
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
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rcutils/logging_macros.h"
#include <pendulum_msgs/msg/rttest_results.hpp>

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

#include "pendulum_motor/pendulum_motor.hpp"
#include "pendulum_motor/visibility_control.h"

using namespace std::chrono_literals;

namespace pendulum
{

class PendulumMotorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit PendulumMotorNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("PendulumMotor", options)
    { };
    COMPOSITION_PUBLIC
    explicit PendulumMotorNode(const std::string & node_name,
            std::chrono::nanoseconds update_period,
            std::chrono::nanoseconds physics_update_period,
            std::unique_ptr<PendulumMotor> motor,
            const rclcpp::NodeOptions & options);
    void on_command_received(const pendulum_msgs::msg::JointCommand::SharedPtr msg);
    void sensor_timer_callback();
    void update_motor_callback();

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
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      pendulum_msgs::msg::JointState>> sensor_pub_;
    std::shared_ptr<rclcpp::Subscription<
      pendulum_msgs::msg::JointCommand>> command_sub_;

    rclcpp::TimerBase::SharedPtr sensor_timer_;
    rclcpp::TimerBase::SharedPtr update_motor_timer_;
    std::chrono::nanoseconds publish_period_ = 1000000ns;
    std::chrono::nanoseconds physics_update_period_ = 1000000ns;
    pendulum_msgs::msg::JointState sensor_message_;
    std::unique_ptr<PendulumMotor> motor_;
};

}  // namespace pendulum

#endif  // PENDULUM__MOTOR_NODE_HPP_
