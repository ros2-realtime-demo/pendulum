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

#include <chrono>
#include <cmath>

#include "pendulum_controller/pendulum_controller.hpp"

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum {

/// Struct for storing PID controller properties.
struct PIDProperties
{
    /// Proportional constant.
    double p = 1;
    /// Integral constant.
    double i = 0;
    /// Derivative constant.
    double d = 0;
    /// Desired state of the plant.
    double command = PI / 2;
};

class PIDController : public PendulumController
{
public:

    PIDController(std::chrono::nanoseconds period, const PIDProperties &pid)
    : publish_period_(period), pid_(pid)
    {
        // Calculate the controller timestep (for discrete differentiation/integration).
        dt_ = publish_period_.count() / (1000.0 * 1000.0 * 1000.0);
        if (std::isnan(dt_) || dt_ == 0) {
            throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
        }
    }
    virtual void update_setpoint_data(const pendulum_msgs::msg::JointCommand &msg) override
    {
      setpoint_position_ = msg.position;
    }

    virtual void update_sensor_data(const pendulum_msgs::msg::JointState &msg) override
    {
      sensor_position_ = msg.position;
    }

    virtual void update_command_data(pendulum_msgs::msg::JointCommand &msg) override
    {
      this->update();
      msg.position = pid_.command;
    }

    virtual void update() override
    {
      if (std::isnan(sensor_position_)) {
          throw std::runtime_error("Sensor value was NaN in on_sensor_message callback");
      }
      // PID controller algorithm
      double error = setpoint_position_ - sensor_position_;
      // Proportional gain is proportional to error
      double p_gain = pid_.p * error;
      // Integral gain is proportional to the accumulation of error
      i_gain_ = pid_.i * (i_gain_ + error * dt_);
      // Differential gain is proportional to the change in error
      double d_gain = pid_.d * (error - last_error_) / dt_;
      last_error_ = error;

      // Calculate the message based on PID gains
      pid_.command = sensor_position_ + p_gain + i_gain_ + d_gain;
      // Enforce positional limits
      if (pid_.command > PI) {
          pid_.command = PI;
      } else if (pid_.command < 0) {
          pid_.command = 0;
      }

      if (std::isnan(pid_.command)) {
          throw std::runtime_error("Resulting command was NaN in on_sensor_message callback");
      }
    }

private:
    std::chrono::nanoseconds publish_period_;
    PIDProperties pid_;
    double last_error_ = 0;
    double i_gain_ = 0;
    double dt_;
    double setpoint_position_ = PI/2;
    double sensor_position_ = PI/2;
};

} // namespace pendulum
