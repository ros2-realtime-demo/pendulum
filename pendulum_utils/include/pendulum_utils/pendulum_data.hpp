// Copyright 2021 Carlos San Vicente
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

#ifndef PENDULUM_UTILS__PENDULUM_DATA_HPP_
#define PENDULUM_UTILS__PENDULUM_DATA_HPP_

#include <cmath>


namespace pendulum
{
namespace utils
{

/// Struct representing the dynamic/kinematic state of the pendulum.
struct PendulumState
{
  // Position of the cart in meters
  double cart_position = 0.0;
  // Velocity of the cart in meters/s
  double cart_velocity = 0.0;
  // Angular position of the pendulum in radians
  // PI is up position
  double pole_angle = M_PI;
  // angular velocity of the pendulum in rad/s
  double pole_velocity = 0.0;
  // total force applied to the cart in Newton
  double cart_force = 0.0;

  // force applied by the controller
  // this can be considered as the force applied by the cart motor
  double controller_force_ = 0.0;
  // external disturbance force
  // this can be considered as something pushing the cart
  double disturbance_force_ = 0.0;
};

/// Struct representing the dynamic/kinematic state of the pendulum.
struct PendulumReference
{
  // Position of the cart in meters
  double cart_position = 0.0;
  // Velocity of the cart in meters/s
  double cart_velocity = 0.0;
  // Angular position of the pendulum in radians
  // PI is up position
  double pole_angle = M_PI;
  // angular velocity of the pendulum in rad/s
  double pole_velocity = 0.0;
  // total force applied to the cart in Newton
  double cart_force = 0.0;
};


}  // namespace utils
}  // namespace pendulum
#endif  // PENDULUM_UTILS__PENDULUM_DATA_HPP_
