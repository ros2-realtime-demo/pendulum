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
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pendulum_utils/process_settings.hpp"
#include "pendulum_utils/lifecycle_autostart.hpp"

int main(int argc, char * argv[])
{
  int32_t ret = 0;

  try {
    rclcpp::init(argc, argv);

    // Create a static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // Create pendulum controller node
    using pendulum::pendulum_controller::PendulumControllerNode;
    const auto controller_node_ptr =
      std::make_shared<PendulumControllerNode>("pendulum_controller");

    exec.add_node(controller_node_ptr->get_node_base_interface());

    // Create pendulum simulation
    using pendulum::pendulum_driver::PendulumDriverNode;
    const auto driver_node_ptr = std::make_shared<PendulumDriverNode>("pendulum_driver");

    exec.add_node(driver_node_ptr->get_node_base_interface());

    auto controller_rt_cb = controller_node_ptr->get_realtime_callback_group();
    auto driver_rt_cb = driver_node_ptr->get_realtime_callback_group();

    auto thread = std::thread([&exec]() {
      // spin will block until work comes in, execute work as it becomes available, and keep blocking.
      // It will only be interrupted by Ctrl-C.
      exec.spin();
    });

    // Create a static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec_rt;
    exec_rt.add_callback_group(controller_rt_cb, controller_node_ptr->get_node_base_interface());
    exec_rt.add_callback_group(driver_rt_cb, driver_node_ptr->get_node_base_interface());

    exec_rt.spin();

    rclcpp::shutdown();
    thread.join();
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(
      rclcpp::get_logger("pendulum_demo"), "Unknown exception caught. "
      "Exiting...");
    ret = -1;
  }
  return ret;
}
