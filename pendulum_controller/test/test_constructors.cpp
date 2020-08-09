// Copyright 2020 Igor Recio
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

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pendulum_controller/pendulum_controller.hpp"

using pendulum::pendulum_controller::PendulumControllerNode;
using pendulum::pendulum_controller::PendulumController;

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

TEST(ConstructorsTest, test_constructors) {
  rclcpp::init(0, nullptr);
  PendulumController::Config config({0.0, 0.0, 0.0, 0.0});

  const auto test_node_ptr =
    std::make_shared<PendulumControllerNode>(
    "test_node", "sensor_topic_name",
    "command_topic_name", "setpoint_topic_name",
    std::chrono::microseconds(1000), config);

  auto names = test_node_ptr->get_node_names();

  EXPECT_EQ(names.size(), 1u);
  EXPECT_STREQ(names[0].c_str(), "/test_node");
  EXPECT_STREQ("/", test_node_ptr->get_namespace());
}

TEST(TransitionTest, test_transition) {
  PendulumController::Config config({0.0, 0.0, 0.0, 0.0});
  const auto test_node =
    std::make_shared<PendulumControllerNode>(
    "test_node", "sensor_topic_name",
    "command_topic_name", "setpoint_topic_name",
    std::chrono::microseconds(1000), config);

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_ACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_FINALIZED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());
}

TEST(ConfigTest, test_config) {
  std::vector<double> feedback_matrix = {0.0, 0.0, 0.0, 0.0};
  PendulumController::Config config({0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(feedback_matrix, config.get_feedback_matrix());
}

TEST(ParamTest, test_param) {
  std::vector<rclcpp::Parameter> params;
  std::vector<double> feedback_matrix = {-10.0000, -51.5393, 356.8637, 154.4146};

  params.emplace_back("sensor_topic_name", "joint_states");
  params.emplace_back("command_topic_name", "command");
  params.emplace_back("setpoint_topic_name", "setpoint");
  params.emplace_back("command_publish_period_us", 10000);
  params.emplace_back("controller.feedback_matrix", feedback_matrix);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  const auto test_node_ptr =
    std::make_shared<PendulumControllerNode>(node_options);
}
