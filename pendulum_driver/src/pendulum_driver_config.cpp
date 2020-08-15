// Copyright 2020 Carlos San Vicente
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

#include "pendulum_driver/pendulum_driver.hpp"
#include <array>

namespace pendulum
{
namespace pendulum_driver
{

PendulumDriver::Config::Config(
  const double pendulum_mass,
  const double cart_mass,
  const double pendulum_length,
  const double damping_coefficient,
  const double gravity,
  const double max_cart_force,
  const double noise_level,
  std::chrono::microseconds physics_update_period)
: pendulum_mass{pendulum_mass},
  cart_mass{cart_mass},
  pendulum_length{pendulum_length},
  damping_coefficient{damping_coefficient},
  gravity{gravity},
  max_cart_force{max_cart_force},
  noise_level{noise_level},
  physics_update_period{physics_update_period}
{}

double PendulumDriver::Config::get_pendulum_mass() const
{
  return pendulum_mass;
}

double PendulumDriver::Config::get_cart_mass() const
{
  return cart_mass;
}

double PendulumDriver::Config::get_pendulum_length() const
{
  return pendulum_length;
}

double PendulumDriver::Config::get_damping_coefficient() const
{
  return damping_coefficient;
}

double PendulumDriver::Config::get_gravity() const
{
  return gravity;
}

double PendulumDriver::Config::get_max_cart_force() const
{
  return max_cart_force;
}

double PendulumDriver::Config::get_noise_level() const
{
  return noise_level;
}

std::chrono::microseconds PendulumDriver::Config::get_physics_update_period() const
{
  return physics_update_period;
}

}  // namespace pendulum_driver
}  // namespace pendulum
