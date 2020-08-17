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
#include "rclcpp/rclcpp.hpp"
#include "pendulum_driver/pendulum_driver.hpp"

using pendulum::pendulum_driver::PendulumDriver;

TEST(ConstructorOptionsTest, test_options_constructor) {
  rclcpp::init(0, nullptr);

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
