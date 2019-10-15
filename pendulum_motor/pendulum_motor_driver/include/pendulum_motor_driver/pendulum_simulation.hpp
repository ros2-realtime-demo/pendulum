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

#ifndef PENDULUM_MOTOR_DRIVER__PENDULUM_SIMULATION_HPP_
#define PENDULUM_MOTOR_DRIVER__PENDULUM_SIMULATION_HPP_
#include <cmath>
#include <chrono>
#include <vector>
#include <random>

#include "rttest/rttest.h"
#include "rttest/utils.h"

#include "pendulum_motor_driver/pendulum_motor_driver.hpp"
#include "pendulum_motor_driver/runge_kutta.hpp"

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum
{

/// Struct representing the dynamic/kinematic state of the pendulum.
struct PendulumState
{
  double cart_position = 0.0;
  double cart_velocity = 0.0;
  double pole_angle = PI;  // UP
  double pole_velocity = 0.0;
};

class PendulumMotorSim : public PendulumMotor
{
public:
  explicit PendulumMotorSim(std::chrono::nanoseconds physics_update_period)
  : physics_update_period_(physics_update_period), done_(false),
    integrator_(4), X_{0.0, 0.0, PI, 0.0},
    rand_gen_(rd()), noise_gen_(std::uniform_real_distribution<double>(-0.01, 0.01))
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
    pthread_attr_setschedpolicy(&thread_attr_, SCHED_FIFO);
    pthread_create(&physics_update_thread_, &thread_attr_,
      &pendulum::PendulumMotorSim::physics_update_wrapper, this);
  }

  virtual void update_command_data(const pendulum_msgs_v2::msg::PendulumCommand & msg)
  {
    controller_force_ = msg.cart_force;
  }

  virtual void update_sensor_data(pendulum_msgs_v2::msg::PendulumState & msg)
  {
    msg.cart_position = state_.cart_position;
    msg.cart_velocity = state_.cart_velocity;
    msg.pole_angle = state_.pole_angle;
    msg.pole_velocity = state_.pole_velocity;
  }

  virtual void update()
  {
  }

  virtual bool init() {}
  virtual bool start() {}
  virtual bool stop() {}

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
    auto derivative_function = [this](const std::vector<double> &y,
       double u, size_t i) -> double {
      if (i == 0) {
        return y[1];
      } else if (i == 1) {
        double Sy = sin(y[2]);
        double Cy = cos(y[2]);
        double D = m*L*L*(M+m*(1-Cy*Cy));
        return (1/D)*(-m*m*L*L*g*Cy*Sy + m*L*L*(m*L*y[3]*y[3]*Sy - d*y[1])) + m*L*L*(1/D)*u;
      } else if (i == 2) {
        return y[3];
      } else if (i == 3) {
        double Sy = sin(y[2]);
        double Cy = cos(y[2]);
        double D = m*L*L*(M+m*(1-Cy*Cy));
        return (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y[3]*y[3]*Sy
           - d*y[1])) - m*L*Cy*(1/D)*u +noise_gen_(rand_gen_);
      } else {
          throw std::invalid_argument("received wrong index");
      }
    };

    rttest_lock_and_prefault_dynamic();

    while (!done_) {
      double cart_force = disturbance_force_ + controller_force_;
      integrator_.step(derivative_function, X_, dt_, cart_force);

      state_.cart_position = X_[0];
      state_.cart_velocity = X_[1];
      state_.pole_angle = X_[2];
      state_.pole_velocity = X_[3];
      // high resolution sleep
      clock_nanosleep(CLOCK_MONOTONIC, 0, &physics_update_timespec_, NULL);
    }
    return 0;
  }

private:
  std::chrono::nanoseconds physics_update_period_;
  double dt_;
  PendulumState state_;
  bool done_ = false;

  pthread_t physics_update_thread_;
  pthread_attr_t thread_attr_;
  timespec physics_update_timespec_;

  RungeKutta integrator_;
  double controller_force_ = 0.0;
  double disturbance_force_ = 0.0;
  std::vector<double> X_;

  double m = 1.0;
  double M = 5.0;
  double L = 2.0;
  double g = -9.8;
  double d = 20.0;

  std::random_device rd;
  std::mt19937 rand_gen_;
  std::uniform_real_distribution<double> noise_gen_;
};

}  // namespace pendulum
#endif  // PENDULUM_MOTOR_DRIVER__PENDULUM_SIMULATION_HPP_
