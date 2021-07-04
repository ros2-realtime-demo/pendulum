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

#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_utils/process_settings.hpp"
#include "pendulum_utils/lifecycle_autostart.hpp"

using pendulum_driver::PendulumDriverNode;
using utils::ProcessSettings;
using utils::configure_process_priority;
using utils::lock_process_memory;

int main(int argc, char * argv[])
{
  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);

    // Create pendulum simulation
    const auto driver_node_ptr = std::make_shared<PendulumDriverNode>("pendulum_driver");
    ProcessSettings rt_settings = driver_node_ptr->get_proc_settings();

    // Create a static executor to run non-real time tasks
    rclcpp::executors::StaticSingleThreadedExecutor exec;
    exec.add_node(driver_node_ptr->get_node_base_interface());
    auto thread = std::thread([&exec]() {exec.spin();});

    // Create a thread to run real-time tasks
    auto rt_thread = std::thread(
      [&driver_node_ptr, &rt_settings]() {
        configure_process_priority(rt_settings.process_priority, rt_settings.cpu_affinity);
        driver_node_ptr->run_realtime_loop();
      });

    if (rt_settings.lock_memory) {
      lock_process_memory(rt_settings.lock_memory_size_mb);
    }

    driver_node_ptr->start();

    // TODO(carlosvg): add wait loop or experiment duration option
    rclcpp::sleep_for(std::chrono::seconds(3600));

    rclcpp::shutdown();
    thread.join();
    rt_thread.join();
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_driver"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_driver"), "Unknown exception caught. Exiting...");
    ret = -1;
  }
  return ret;
}
