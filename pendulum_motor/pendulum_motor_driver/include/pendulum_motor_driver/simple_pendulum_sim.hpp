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

#include <cmath>
#include <chrono>

#include "rttest/rttest.h"
#include "rttest/utils.h"

#include "pendulum_motor_driver/pendulum_motor_driver.hpp"

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum {

  /// Struct representing the physical properties of the pendulum.
  struct PendulumProperties
  {
    // Mass of the weight on the end of the pendulum in kilograms
    double mass = 0.01;
    // Length of the pendulum in meters
    double length = 0.5;
  };

  /// Struct representing the dynamic/kinematic state of the pendulum.
  struct PendulumState
  {
    // Angle from the ground in radians
    double position = PI / 2;
    // Angular velocity in radians/sec
    double velocity = 0;
    // Angular acceleration in radians/sec^2
    double acceleration = 0;
    // Torque on the joint (currently unused)
    double torque = 0;
  };

class PendulumMotorSim : public PendulumMotor
{
public:

    explicit PendulumMotorSim(std::chrono::nanoseconds physics_update_period)
    : physics_update_period_(physics_update_period), done_(false)
    {
        // Calculate the controller timestep (for discrete differentiation/integration).
        dt_ = physics_update_period_.count() / (1000.0 * 1000.0 * 1000.0);
        if (std::isnan(dt_) || dt_ == 0) {
            throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
        }
        long_to_timespec(physics_update_period_.count(), &physics_update_timespec_);

        // Initialize a separate high-priority thread to run the physics update loop.
        pthread_attr_init(&thread_attr_);
        sched_param thread_param;
        thread_param.sched_priority = 90;
        pthread_attr_setschedparam(&thread_attr_, &thread_param);
        pthread_attr_setschedpolicy(&thread_attr_, SCHED_RR);
        pthread_create(&physics_update_thread_, &thread_attr_,
          &pendulum::PendulumMotorSim::physics_update_wrapper, this);
    }

    virtual void update_command_data(const pendulum_msgs::msg::JointCommand &msg)
    {
      // Assume direct, instantaneous position control
      // (It would be more realistic to simulate a motor model)
      state_.torque = msg.position;

      // // Enforce position limits
      // if (state_.position > PI) {
      //   state_.position = PI;
      // } else if (state_.position < 0) {
      //   state_.position = 0;
      // }
    }

    virtual void update_sensor_data(pendulum_msgs::msg::JointState &msg)
    {
      msg.velocity = state_.velocity;
      msg.position = state_.position;
      msg.effort = state_.acceleration;
    }

    virtual void update()
    {
      state_.acceleration = GRAVITY;
        //GRAVITY * std::sin(state_.position - PI / 2.0) / properties_.length;// +
        //state_.torque / (properties_.mass * properties_.length * properties_.length);
      state_.velocity += state_.acceleration * dt_;
      state_.position += state_.velocity * dt_;
      // if (state_.position > PI) {
      //   state_.position = PI;
      // } else if (state_.position < 0) {
      //   state_.position = 0;
      // }
    }
    /// Set the boolean to signal that the physics engine should finish.
    // \param[in] done True if the physics engine should stop.
    void set_done(bool done)
    {
      done_ = done;
    }

    /// Get the status of the physics engine.
    // \return True if the physics engine is running, false otherwise.
    bool done() const
    {
      return done_;
    }

private:

  static void * physics_update_wrapper(void * args)
  {
    PendulumMotorSim * motor = static_cast<PendulumMotorSim *>(args);
    if (!motor) {
      return NULL;
    }
    return motor->physics_update();
  }
  // Set kinematic and dynamic properties of the pendulum based on state inputs
  void * physics_update()
  {

    rttest_lock_and_prefault_dynamic();
    while (!done_) {
      state_.acceleration = GRAVITY * std::sin(state_.position - PI / 2.0) / properties_.length;
      // +
      //  state_.torque / (properties_.mass * properties_.length * properties_.length);
      state_.velocity += state_.acceleration * dt_;
      state_.position += state_.velocity * dt_;
      // if (state_.position > PI) {
      //   state_.position = PI;
      // } else if (state_.position < 0) {
      //   state_.position = 0;
      // }

      if (std::isnan(state_.position)) {
        throw std::runtime_error("Tried to set state to NaN in on_command_message callback");
      }

      // high resolution sleep
      clock_nanosleep(CLOCK_MONOTONIC, 0, &physics_update_timespec_, NULL);
    }
    return 0;
  }


private:
    std::chrono::nanoseconds physics_update_period_;
    double dt_;
    PendulumProperties properties_;
    PendulumState state_;
    bool done_ = false;

    pthread_t physics_update_thread_;
    pthread_attr_t thread_attr_;
    timespec physics_update_timespec_;
};

} // namespace pendulum
