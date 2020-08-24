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
TEST(ConstructorTest, test_constructor) {
}

// Test update functions & reset
TEST(UpdateTest, test_update_reset) {
  PendulumController::Config config({-10.0000, -51.5393, 356.8637, 154.4146});
  PendulumController controller(config);

  pendulum2_msgs::msg::PendulumTeleop msg_teleop;
  sensor_msgs::msg::JointState msg_status;
  pendulum2_msgs::msg::JointCommandStamped msg_command;

  controller.update_teleop_data(msg_teleop);
  // controller.update_status_data(msg_status);
  // controller.update_command_data(msg_command);
  // controller.reset();
}

// Test calculate
TEST(CalculateTest, test_calculate) {
  std::vector<double> state;
  std::vector<double> reference;
}
