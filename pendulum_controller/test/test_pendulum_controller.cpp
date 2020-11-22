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
#include "pendulum_controller/pendulum_controller.hpp"
#include "apex_test_tools/apex_test_tools.hpp"

// TODO(carlossvg): add out of range tests

using pendulum::pendulum_controller::PendulumController;

class TestPendulumController : public ::testing::Test
{
protected:
  std::vector<double> feedback_matrix = {-10.0000, -51.5393, 356.8637, 154.4146};
  PendulumController::Config config{feedback_matrix};
};

TEST_F(TestPendulumController, config)
{
  std::vector<double> test_feedback_matrix(4);

  apex_test_tools::memory_test::start();
  test_feedback_matrix = config.get_feedback_matrix();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(test_feedback_matrix.at(0), feedback_matrix.at(0));
  EXPECT_FLOAT_EQ(test_feedback_matrix.at(1), feedback_matrix.at(1));
  EXPECT_FLOAT_EQ(test_feedback_matrix.at(2), feedback_matrix.at(2));
  EXPECT_FLOAT_EQ(test_feedback_matrix.at(3), feedback_matrix.at(3));
}

TEST_F(TestPendulumController, set_teleoperation)
{
  PendulumController controller{config};
  double cart_pos{1.0};
  double cart_vel{2.0};
  double pole_pos{3.0};
  double pole_vel{4.0};
  std::vector<double> teleop_data(4);

  apex_test_tools::memory_test::start();
  controller.set_teleop(cart_pos, cart_vel, pole_pos, pole_vel);
  teleop_data = controller.get_teleop();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(cart_pos, teleop_data.at(0));
  EXPECT_FLOAT_EQ(cart_vel, teleop_data.at(1));
  EXPECT_FLOAT_EQ(pole_pos, teleop_data.at(2));
  EXPECT_FLOAT_EQ(pole_vel, teleop_data.at(3));

  cart_pos = 5.0;
  cart_vel = 6.0;

  apex_test_tools::memory_test::start();
  controller.set_teleop(cart_pos, cart_vel);
  teleop_data = controller.get_teleop();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(cart_pos, teleop_data.at(0));
  EXPECT_FLOAT_EQ(cart_vel, teleop_data.at(1));
}

TEST_F(TestPendulumController, set_state)
{
  PendulumController controller{config};
  double cart_pos{1.0};
  double cart_vel{2.0};
  double pole_pos{3.0};
  double pole_vel{4.0};
  std::vector<double> state_data(4);

  apex_test_tools::memory_test::start();
  controller.set_state(cart_pos, cart_vel, pole_pos, pole_vel);
  state_data = controller.get_state();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(cart_pos, state_data.at(0));
  EXPECT_FLOAT_EQ(cart_vel, state_data.at(1));
  EXPECT_FLOAT_EQ(pole_pos, state_data.at(2));
  EXPECT_FLOAT_EQ(pole_vel, state_data.at(3));
}

TEST_F(TestPendulumController, set_force_command)
{
  PendulumController controller{config};
  double expected_force{1.0};

  apex_test_tools::memory_test::start();
  controller.set_force_command(expected_force);
  double returned_force = controller.get_force_command();
  apex_test_tools::memory_test::stop();

  EXPECT_FLOAT_EQ(expected_force, returned_force);
}

TEST_F(TestPendulumController, init_state)
{
  PendulumController controller{config};

  auto state_data = controller.get_state();
  EXPECT_FLOAT_EQ(0.0, state_data.at(0));
  EXPECT_FLOAT_EQ(0.0, state_data.at(1));
  EXPECT_FLOAT_EQ(M_PI, state_data.at(2));
  EXPECT_FLOAT_EQ(0.0, state_data.at(3));

  auto teleop_data = controller.get_teleop();
  EXPECT_FLOAT_EQ(0.0, teleop_data.at(0));
  EXPECT_FLOAT_EQ(0.0, teleop_data.at(1));
  EXPECT_FLOAT_EQ(M_PI, teleop_data.at(2));
  EXPECT_FLOAT_EQ(0.0, teleop_data.at(3));
}

TEST_F(TestPendulumController, reset)
{
  PendulumController controller{config};
  controller.set_state(1.0, 2.0, 3.0, 4.0);
  controller.set_teleop(5.0, 6.0, 7.0, 8.0);

  apex_test_tools::memory_test::start();
  controller.reset();
  apex_test_tools::memory_test::stop();

  auto state_data = controller.get_state();
  EXPECT_FLOAT_EQ(0.0, state_data.at(0));
  EXPECT_FLOAT_EQ(0.0, state_data.at(1));
  EXPECT_FLOAT_EQ(M_PI, state_data.at(2));
  EXPECT_FLOAT_EQ(0.0, state_data.at(3));

  auto teleop_data = controller.get_teleop();
  EXPECT_FLOAT_EQ(0.0, teleop_data.at(0));
  EXPECT_FLOAT_EQ(0.0, teleop_data.at(1));
  EXPECT_FLOAT_EQ(M_PI, teleop_data.at(2));
  EXPECT_FLOAT_EQ(0.0, teleop_data.at(3));
}

TEST_F(TestPendulumController, update)
{
  PendulumController controller{config};
  apex_test_tools::memory_test::start();
  controller.update();
  apex_test_tools::memory_test::stop();
}
