// Copyright 2017 Apex.AI, Inc.
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

#ifndef PENDULUM_TOOLS__STATISTICS_TRACKER_HPP_
#define PENDULUM_TOOLS__STATISTICS_TRACKER_HPP_

#include <vector>
#include <limits>
#include <cmath>

namespace pendulum
{
/// Calculates statistical metrics out of incrementally added samples.
class StatisticsTracker
{
public:
  StatisticsTracker()
  : m_min(std::numeric_limits<double>::max()),
    m_max(std::numeric_limits<double>::lowest()),
    m_n(0.0),
    m_mean(0.0),
    m_M2(0.0)
  {
    static_assert(std::numeric_limits<double>::is_iec559, "Non IEEE754 are not supported.");
  }
  /**
   * \brief Fusions multiple StatisticsTrackers.
   * \param st_vec Vector of elements to fusions.
   */
  explicit StatisticsTracker(std::vector<StatisticsTracker> st_vec)
  : m_min(std::numeric_limits<double>::max()),
    m_max(std::numeric_limits<double>::lowest()),
    m_n(0.0),
    m_mean(0.0),
    m_M2(0.0)
  {
    if (st_vec.empty()) {
      return;
    }
    if (st_vec.size() == 1) {
      const auto & a = st_vec.at(0);
      m_min = a.min();
      m_max = a.max();
      m_n = a.m_n;
      m_mean = a.m_mean;
      m_M2 = a.m_M2;
      return;
    }
    double mean_t = 0.0, n_total = 0.0;
    for (const auto & a : st_vec) {
      if (a.min() < m_min) {
        m_min = a.min();
      }
      if (a.max() > m_max) {
        m_max = a.max();
      }

      n_total += a.n();
      mean_t += a.n() * a.mean();
    }
    m_n = n_total;
    m_mean = mean_t / n_total;

    // Algorithm taken from https://www.geeksforgeeks.org/find-combined-mean-variance-two-series/
    double var_t = 0.0;
    for (const auto & a : st_vec) {
      const auto d2 = (a.mean() - m_mean) * (a.mean() - m_mean);
      var_t += a.n() * (a.variance() + d2);
    }
    var_t = var_t / n_total;
    m_M2 = var_t * (n_total - 1.0);
  }
  /// Adds a sample to consider in the metrics.
  void add_sample(const double x)
  {
    // Algorithms taken from
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Higher-order_statistics.
    if (x < m_min) {
      m_min = x;
    }
    if (x > m_max) {
      m_max = x;
    }

    const auto n1 = m_n;
    m_n = m_n + 1.0;

    const auto delta = x - m_mean;

    const auto delta_n = delta / m_n;
    const auto term1 = delta * delta_n * n1;
    m_mean = m_mean + delta_n;
    m_M2 = m_M2 + term1;
  }
  /// The number of all samples added.
  double n() const
  {
    return m_n;
  }

  /// The minimum value over all samples added.
  double min() const
  {
    return m_min;
  }

  /// The maximum value over all samples added.
  double max() const
  {
    return m_max;
  }

  /// The mean over all samples added.
  double mean() const
  {
    return m_mean;
  }

  /// The variance over all samples added.
  double variance() const
  {
    return m_M2 / m_n;
  }

private:
  double m_min;
  double m_max;
  double m_n;
  double m_mean;
  double m_M2;
};

}  // namespace pendulum

#endif  // PENDULUM_TOOLS__STATISTICS_TRACKER_HPP_
