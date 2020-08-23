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
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_driver/pendulum_driver.hpp"

using pendulum::pendulum_driver::PendulumDriver;
using pendulum::pendulum_driver::PendulumDriverNode;

class InitNodesTest : public ::testing::Test
{
protected:
  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;

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
    params.emplace_back("disturbance_topic_name", "disturbance");
    params.emplace_back("cart_base_joint_name", "cart_base_joint");
    params.emplace_back("pole_joint_name", "pole_joint");
    params.emplace_back("state_publish_period_us", 10000);
    params.emplace_back("enable_topic_stats", false);
    params.emplace_back("topic_stats_topic_name", "driver_stats");
    params.emplace_back("topic_stats_publish_period_ms", 1000);
    params.emplace_back("deadline_duration_ms", 0);
    params.emplace_back("driver.pendulum_mass", 1.0);
    params.emplace_back("driver.cart_mass", 5.0);
    params.emplace_back("driver.pendulum_length", 2.0);
    params.emplace_back("driver.damping_coefficient", 20.0);
    params.emplace_back("driver.gravity", -9.8);
    params.emplace_back("driver.max_cart_force", 1000.0);
    params.emplace_back("driver.physics_update_period", 10000);
    params.emplace_back("driver.noise_level", 1.0);
  }

  static void TearDownTestCase()
  {
    (void)rclcpp::shutdown();
  }
};

TEST(ConfigTest, test_config_driver) {
  PendulumDriver::Config config(1.0, 5.0, 2.0, 20.0, -9.8,
    1000.0,
    1.0,
    std::chrono::microseconds(1000));

  EXPECT_EQ(config.get_pendulum_mass(), 1.0);
  EXPECT_EQ(config.get_cart_mass(), 5.0);
  EXPECT_EQ(config.get_pendulum_length(), 2.0);
  EXPECT_EQ(config.get_damping_coefficient(), 20.0);
  EXPECT_EQ(config.get_gravity(), -9.8);
  EXPECT_EQ(config.get_max_cart_force(), 1000.0);
  EXPECT_EQ(config.get_noise_level(), 1.0);
  EXPECT_EQ(config.get_physics_update_period(), std::chrono::microseconds(1000));
}

TEST_F(InitNodesTest, test_constructors) {
  node_options.parameter_overrides(params);

  const auto test_node_ptr =
    std::make_shared<PendulumDriverNode>(node_options);
  auto names = test_node_ptr->get_node_names();

  EXPECT_EQ(names.size(), 1u);
  EXPECT_STREQ(names[0].c_str(), "/pendulum_driver");
  EXPECT_STREQ("/", test_node_ptr->get_namespace());
}
