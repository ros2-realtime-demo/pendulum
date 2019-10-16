// Copyright 2019 Carlos San Vicente
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

#ifndef PENDULUM_CONTROLLERS__FULL_STATE_FEEDBACK_CONTROLLER_HPP_
#define PENDULUM_CONTROLLERS__FULL_STATE_FEEDBACK_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <vector>

#include "pendulum_controller_node/pendulum_controller.hpp"

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum
{
// Implements a Full State Feedback controller
// https://en.wikipedia.org/wiki/Full_state_feedback
class FullStateFeedbackController : public PendulumController
{
public:
  FullStateFeedbackController(
    std::chrono::nanoseconds period,
    const std::vector<double> & feedback_matrix);

  void update_setpoint_data(const pendulum_msgs_v2::msg::PendulumCommand & msg) override;
  void update_sensor_data(const pendulum_msgs_v2::msg::PendulumState & msg) override;
  void update_command_data(pendulum_msgs_v2::msg::PendulumCommand & msg) override;
  void update() override;
  void reset() override;

private:
  double calculate(
    const std::vector<double> & state,
    const std::vector<double> & reference) const;

private:
  std::chrono::nanoseconds publish_period_;
  std::vector<double> feedback_matrix_;
  std::vector<double> state_;
  std::vector<double> reference_;
};
}  // namespace pendulum
#endif  // PENDULUM_CONTROLLERS__FULL_STATE_FEEDBACK_CONTROLLER_HPP_
