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

#ifndef PENDULUM_SIMULATION__RUNGE_KUTTA_HPP_
#define PENDULUM_SIMULATION__RUNGE_KUTTA_HPP_

#include <vector>
#include <stdexcept>
#include <functional>

namespace pendulum
{

using derivativeF = std::function<double (const std::vector<double> &, double, size_t)>;

class RungeKutta
{
public:
  explicit RungeKutta(size_t dimension)
  : n(dimension)
  {
    k1.resize(dimension);
    k2.resize(dimension);
    k3.resize(dimension);
    k4.resize(dimension);
    state.resize(dimension);
  }

  // Time step using 4th-orderRunge Kutta and trapezoidal rule
  // See: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
  void step(derivativeF df, std::vector<double> & y, double h, double u)
  {
    if (y.size() != n) {
      throw std::invalid_argument("wrong state size vector");
    }

    size_t i = 0;

    // stage 1
    for (i = 0; i < n; i++) {
      k1[i] = df(y, u, i);
    }

    // stage 2
    for (i = 0; i < n; i++) {
      state[i] = y[i] + h * 0.5 * k1[i];
    }
    for (i = 0; i < n; i++) {
      k2[i] = df(state, u, i);
    }

    // stage 3
    for (i = 0; i < n; i++) {
      state[i] = y[i] + h * (0.5 * k2[i]);
    }
    for (i = 0; i < n; i++) {
      k3[i] = df(state, u, i);
    }

    // stage 4
    for (i = 0; i < n; i++) {
      state[i] = y[i] + h * (1.0 * k3[i]);
    }
    for (i = 0; i < n; i++) {
      k4[i] = df(state, u, i);
    }

    // update next step
    for (i = 0; i < n; i++) {
      y[i] = y[i] + (h / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
  }

private:
  std::vector<double> state;
  std::vector<double> k1, k2, k3, k4;
  size_t n;
};

}  // namespace pendulum

#endif  // PENDULUM_SIMULATION__RUNGE_KUTTA_HPP_
