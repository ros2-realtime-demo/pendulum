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

/// \file
/// \brief This file provides an implementation for a Runge-Kutta method to solve
///        ordinary differential equations (ODE)

#ifndef PENDULUM_DRIVER__RUNGE_KUTTA_HPP_
#define PENDULUM_DRIVER__RUNGE_KUTTA_HPP_

#include <array>
#include <stdexcept>
#include <functional>

namespace pendulum
{
namespace pendulum_driver
{
template<std::size_t N>
using derivativeF = std::function<double (const std::array<double, N> &, double, size_t)>;

/// \class This class implements a classic 4th order
/// <a href="https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods"> Runge Kutta method</a>
/// compilation
/// This method is based on the trapezoidal rule and it just allocates memory at initialization.
template<std::size_t N>
class RungeKutta
{
public:
  /// \brief Time step using 4th-orderRunge Kutta and trapezoidal rule
  /// \param[in] df Derivative function pointing to the ODE equations to solve
  /// \param[in,out] y Status vector with the previous status at input and next state at output.
  /// \param[in] h Time step.
  /// \param[in] u Single input in the equations.
  /// \throw std::invalid_argument If the state vector doesn't has wrong dimensions.
  void step(derivativeF<N> df, std::array<double, N> & y, double h, double u)
  {
    std::size_t i = 0;

    // stage 1
    for (i = 0; i < N; i++) {
      k1[i] = df(y, u, i);
    }

    // stage 2
    for (i = 0; i < N; i++) {
      state[i] = y[i] + h * 0.5 * k1[i];
    }
    for (i = 0; i < N; i++) {
      k2[i] = df(state, u, i);
    }

    // stage 3
    for (i = 0; i < N; i++) {
      state[i] = y[i] + h * (0.5 * k2[i]);
    }
    for (i = 0; i < N; i++) {
      k3[i] = df(state, u, i);
    }

    // stage 4
    for (i = 0; i < N; i++) {
      state[i] = y[i] + h * (1.0 * k3[i]);
    }
    for (i = 0; i < N; i++) {
      k4[i] = df(state, u, i);
    }

    // update next step
    for (i = 0; i < N; i++) {
      y[i] = y[i] + (h / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
  }

private:
  // state space vector
  // state[0]: cart position
  // state[1]: cart velocity
  // state[2]: pole position
  // state[3]: pole velocity
  std::array<double, N> state;

  // Runge-kutta increments
  std::array<double, N> k1, k2, k3, k4;
};
}  // namespace pendulum_driver
}  // namespace pendulum

#endif  // PENDULUM_DRIVER__RUNGE_KUTTA_HPP_
