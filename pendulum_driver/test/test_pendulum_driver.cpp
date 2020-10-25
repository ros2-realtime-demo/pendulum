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
#include "pendulum_driver/pendulum_driver.hpp"
#include "apex_test_tools/apex_test_tools.hpp"

// TODO(carlossvg): add out of range tests

using pendulum::pendulum_driver::PendulumDriver;

class TestPendulumDriver : public ::testing::Test
{
protected:
  double pendulum_mass{1.0};
  double cart_mass{5.0};
  double pendulum_length{2.0};
  double damping_coefficient{20.0};
  double gravity{-9.8};
  double max_cart_force{1000.0};
  double noise_level{1.0};
  std::chrono::microseconds state_publish_period{1000};
  PendulumDriver::Config config{pendulum_mass, cart_mass, pendulum_length, damping_coefficient,
    gravity, max_cart_force, noise_level, state_publish_period};
};

TEST_F(TestPendulumDriver, config)
{
  double test_pendulum_mass{0.0};
  double test_cart_mass{0.0};
  double test_pendulum_length{0.0};
  double test_damping_coefficient{0.0};
  double test_gravity{0.0};
  double test_max_cart_force{0.0};
  double test_noise_level{0.0};
  std::chrono::microseconds test_state_publish_period{0};

  apex_test_tools::memory_test::start();
  test_pendulum_mass = config.get_pendulum_mass();
  test_cart_mass = config.get_cart_mass();
  test_pendulum_length = config.get_pendulum_length();
  test_damping_coefficient = config.get_damping_coefficient();
  test_gravity = config.get_gravity();
  test_max_cart_force = config.get_max_cart_force();
  test_noise_level = config.get_noise_level();
  test_state_publish_period = config.get_physics_update_period();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(test_pendulum_mass, pendulum_mass);
  EXPECT_FLOAT_EQ(test_cart_mass, cart_mass);
  EXPECT_FLOAT_EQ(test_pendulum_length, pendulum_length);
  EXPECT_FLOAT_EQ(test_damping_coefficient, damping_coefficient);
  EXPECT_FLOAT_EQ(test_gravity, gravity);
  EXPECT_FLOAT_EQ(test_max_cart_force, max_cart_force);
  EXPECT_FLOAT_EQ(test_noise_level, noise_level);
  EXPECT_EQ(test_state_publish_period.count(), state_publish_period.count());
}

TEST_F(TestPendulumDriver, set_state)
{
  PendulumDriver driver{config};
  PendulumDriver::PendulumState state;
  double cart_position{1.0};
  double cart_velocity{2.0};
  double pole_angle{3.0};
  double pole_velocity{4.0};

  apex_test_tools::memory_test::start();
  driver.set_state(cart_position, cart_velocity, pole_angle, pole_velocity);
  state = driver.get_state();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(cart_position, state.cart_position);
  EXPECT_FLOAT_EQ(cart_velocity, state.cart_velocity);
  EXPECT_FLOAT_EQ(pole_angle, state.pole_angle);
  EXPECT_FLOAT_EQ(pole_velocity, state.pole_velocity);
}

TEST_F(TestPendulumDriver, set_controller_cart_force)
{
  PendulumDriver driver{config};
  double expected_force{1.0};

  apex_test_tools::memory_test::start();
  driver.set_controller_cart_force(expected_force);
  double force = driver.get_controller_cart_force();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(expected_force, force);
}

TEST_F(TestPendulumDriver, set_disturbance_force)
{
  PendulumDriver driver{config};
  double expected_force{1.0};

  apex_test_tools::memory_test::start();
  driver.set_disturbance_force(expected_force);
  double force = driver.get_disturbance_force();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(expected_force, force);
}

TEST_F(TestPendulumDriver, init_state)
{
  PendulumDriver driver{config};
  PendulumDriver::PendulumState state;
  double controller_cart_force{0.0};
  double disturbance_force{0.0};

  apex_test_tools::memory_test::start();
  state = driver.get_state();
  controller_cart_force = driver.get_controller_cart_force();
  disturbance_force = driver.get_disturbance_force();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(0.0, state.cart_position);
  EXPECT_FLOAT_EQ(0.0, state.cart_velocity);
  EXPECT_FLOAT_EQ(M_PI, state.pole_angle);
  EXPECT_FLOAT_EQ(0.0, state.pole_velocity);
  EXPECT_FLOAT_EQ(0.0, controller_cart_force);
  EXPECT_FLOAT_EQ(0.0, disturbance_force);
}

TEST_F(TestPendulumDriver, reset)
{
  PendulumDriver driver{config};
  PendulumDriver::PendulumState state;
  double controller_cart_force{0.0};
  double disturbance_force{0.0};
  driver.set_state(1.0, 2.0, 3.0, 4.0);
  driver.set_controller_cart_force(1.0);
  driver.set_disturbance_force(1.0);

  apex_test_tools::memory_test::start();
  driver.reset();
  apex_test_tools::memory_test::stop();

  state = driver.get_state();
  controller_cart_force = driver.get_controller_cart_force();
  disturbance_force = driver.get_disturbance_force();

  EXPECT_FLOAT_EQ(0.0, state.cart_position);
  EXPECT_FLOAT_EQ(0.0, state.cart_velocity);
  EXPECT_FLOAT_EQ(M_PI, state.pole_angle);
  EXPECT_FLOAT_EQ(0.0, state.pole_velocity);
  EXPECT_FLOAT_EQ(0.0, controller_cart_force);
  EXPECT_FLOAT_EQ(0.0, disturbance_force);
}

TEST_F(TestPendulumDriver, update)
{
  PendulumDriver driver{config};
  apex_test_tools::memory_test::start();
  driver.update();
  apex_test_tools::memory_test::stop();
}
