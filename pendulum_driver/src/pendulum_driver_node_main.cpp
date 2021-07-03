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

int main(int argc, char * argv[])
{
  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);

    // Create a static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // Create pendulum simulation
    using pendulum::pendulum_driver::PendulumDriverNode;
    const auto driver_node_ptr = std::make_shared<PendulumDriverNode>("pendulum_driver");

    exec.add_node(driver_node_ptr->get_node_base_interface());

    auto driver_rt_cb = driver_node_ptr->get_realtime_callback_group();
    pendulum::utils::ProcessSettings proc_settings = driver_node_ptr->get_proc_settings();

    auto thread = std::thread([&exec]() {exec.spin();});
    auto rt_thread = std::thread(
      [&driver_node_ptr, &proc_settings]() {
        pendulum::utils::configure_process_priority(
          proc_settings.process_priority,
          proc_settings.cpu_affinity);
        driver_node_ptr->run_realtime_loop();
      });

    if (proc_settings.lock_memory) {
      pendulum::utils::lock_process_memory(proc_settings.lock_memory_size_mb);
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
    RCLCPP_INFO(
      rclcpp::get_logger("pendulum_driver"), "Unknown exception caught. "
      "Exiting...");
    ret = -1;
  }
  return ret;
}
