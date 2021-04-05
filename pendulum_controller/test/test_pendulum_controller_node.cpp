// Copyright 2020 Carlos San Vicente, Igor Recio
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
#include <vector>
#include "pendulum_controller/pendulum_controller_node.hpp"

using pendulum::pendulum_controller::PendulumControllerNode;

// TODO(carlossvg): add bad initialization test
// TODO(carlossvg): add enable_topic_stats test
// TODO(carlossvg): add deadline QoS test
// TODO(carlossvg): add test checking simulation

class TestPendulumControllerNode : public ::testing::Test
{
protected:
  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::vector<double> feedback_matrix = {-10.0000, -51.5393, 356.8637, 154.4146};

  static void SetUpTestCase()
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());
  }

  void SetUp() override
  {
    params.emplace_back("state_topic_name", "joint_states");
    params.emplace_back("command_topic_name", "command");
    params.emplace_back("teleop_topic_name", "setpoint");
    params.emplace_back("command_publish_period_us", 10000);
    params.emplace_back("enable_topic_stats", false);
    params.emplace_back("topic_stats_topic_name", "controller_stats");
    params.emplace_back("topic_stats_publish_period_ms", 1000);
    params.emplace_back("deadline_duration_ms", 0);
    params.emplace_back("controller.feedback_matrix", feedback_matrix);
    node_options.parameter_overrides(params);
  }

  static void TearDownTestCase()
  {
    (void)rclcpp::shutdown();
  }
};

TEST_F(TestPendulumControllerNode, constructor)
{
  const PendulumControllerNode controller_node{node_options};
}

TEST_F(TestPendulumControllerNode, constructor_with_name)
{
  const PendulumControllerNode controller_node{"controller_node", node_options};
}

TEST_F(TestPendulumControllerNode, trigger_lifecycle_transitions) {
  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;
  PendulumControllerNode controller_node{node_options};

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, controller_node.get_current_state().id());
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, controller_node.trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_ACTIVE, controller_node.trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, controller_node.trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED, controller_node.trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_FINALIZED, controller_node.trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());
}

TEST_F(TestPendulumControllerNode, pendulum_state_message_size) {
  EXPECT_TRUE(rosidl_generator_traits::has_bounded_size<pendulum2_msgs::msg::JointState>());
  EXPECT_TRUE(rosidl_generator_traits::has_fixed_size<pendulum2_msgs::msg::JointState>());
}

TEST_F(TestPendulumControllerNode, pendulum_command_message_size) {
  EXPECT_TRUE(rosidl_generator_traits::has_bounded_size<pendulum2_msgs::msg::JointCommand>());
  EXPECT_TRUE(rosidl_generator_traits::has_fixed_size<pendulum2_msgs::msg::JointCommand>());
}
