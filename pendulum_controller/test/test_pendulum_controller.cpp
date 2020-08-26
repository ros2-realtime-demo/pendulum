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
#include "sensor_msgs/msg/joint_state.hpp"
#include "pendulum2_msgs/msg/joint_command_stamped.hpp"
#include "pendulum2_msgs/msg/pendulum_teleop.hpp"

using pendulum::pendulum_controller::PendulumControllerNode;
using pendulum::pendulum_controller::PendulumController;

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class InitNodesTest : public ::testing::Test
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
  }

  static void TearDownTestCase()
  {
    (void)rclcpp::shutdown();
  }
};

// Test config
TEST_F(InitNodesTest, test_config) {
  PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  ASSERT_EQ(feedback_matrix, config.get_feedback_matrix());
}

// Test constructor
TEST_F(InitNodesTest, test_constructor) {
  PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  PendulumController controller(config);

  std::vector<double> state = controller.get_state();
  std::vector<double> reference = controller.get_reference();
  PendulumController::Config configObj = controller.get_config();

  ASSERT_EQ(state[0], 0.0);
  ASSERT_EQ(state[1], 0.0);
  ASSERT_EQ(state[2], M_PI);
  ASSERT_EQ(state[3], 0.0);

  ASSERT_EQ(reference[0], 0.0);
  ASSERT_EQ(reference[1], 0.0);
  ASSERT_EQ(reference[2], M_PI);
  ASSERT_EQ(reference[3], 0.0);

  ASSERT_EQ(configObj.get_feedback_matrix(), config.get_feedback_matrix());
}

// Test update teleop
TEST_F(InitNodesTest, test_update_teleop) {
  PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  PendulumController controller(config);

  pendulum2_msgs::msg::PendulumTeleop msg_teleop;
  msg_teleop.cart_position = M_PI / 2;
  msg_teleop.cart_velocity = 3.0;

  controller.update_teleop_data(msg_teleop);

  std::vector<double> ref = controller.get_reference();
  ASSERT_EQ(ref[0], M_PI / 2);
  ASSERT_EQ(ref[1], 3.0);
  ASSERT_EQ(ref[2], M_PI);
  ASSERT_EQ(ref[3], 0.0);
}

// Test update status
TEST_F(InitNodesTest, test_update_status) {
  PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  PendulumController controller(config);

  sensor_msgs::msg::JointState msg_status;
  msg_status.position.emplace_back(M_PI / 2);
  msg_status.position.emplace_back(M_PI / 2);
  msg_status.velocity.emplace_back(3.0);
  msg_status.velocity.emplace_back(2.0);

  controller.update_status_data(msg_status);

  std::vector<double> state = controller.get_state();
  ASSERT_EQ(state[0], M_PI / 2);
  ASSERT_EQ(state[1], 3.0);
  ASSERT_EQ(state[2], M_PI / 2);
  ASSERT_EQ(state[3], 2.0);
}

// Test update command
TEST_F(InitNodesTest, test_update_command) {
  PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  PendulumController controller(config);

  pendulum2_msgs::msg::JointCommandStamped msg_command;
  controller.reset();
  controller.update_command_data(msg_command);
  ASSERT_EQ(msg_command.cmd.force, 0.0);

  sensor_msgs::msg::JointState msg_status;
  msg_status.position.emplace_back(M_PI / 2);
  msg_status.position.emplace_back(M_PI / 2);
  msg_status.velocity.emplace_back(3.0);
  msg_status.velocity.emplace_back(2.0);
  controller.update_status_data(msg_status);

  pendulum2_msgs::msg::PendulumTeleop msg_teleop;
  msg_teleop.cart_position = M_PI / 2;
  msg_teleop.cart_velocity = 3.0;
  controller.update_teleop_data(msg_teleop);

  controller.update_command_data(msg_command);
  ASSERT_FLOAT_EQ(msg_command.cmd.force, 251.731);
}

// Test reset
TEST_F(InitNodesTest, test_reset) {
  PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  PendulumController controller(config);
  controller.reset();

  std::vector<double> ref = controller.get_reference();
  ASSERT_EQ(ref[0], 0.0);
  ASSERT_EQ(ref[1], 0.0);
  ASSERT_EQ(ref[2], M_PI);
  ASSERT_EQ(ref[3], 0.0);

  std::vector<double> state = controller.get_state();
  ASSERT_EQ(state[0], 0.0);
  ASSERT_EQ(state[1], 0.0);
  ASSERT_EQ(state[2], M_PI);
  ASSERT_EQ(state[3], 0.0);
}

// Test calculate
TEST(CalculateTest, test_calculate) {
  // std::vector<double> state{0.0, 0.0, M_PI, 0.0};
  // std::vector<double> reference{0.0, 0.0, M_PI, 0.0};
  // PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  // PendulumController controller(config);
  // double result = controller.calculate(state, reference);
  // ASSERT_EQ(result, 0.0);
}
