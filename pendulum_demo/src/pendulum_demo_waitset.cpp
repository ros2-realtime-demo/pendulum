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


    // TODO(carlosvg): protect shared variables, using try-lock or std::atomic
    //  (static assert lock free)

    // TODO(carlosvg): configure threads individually
    auto executor_thread = std::thread(
        [&exec]() {
          exec.spin();
        });
    auto driver_realtime_thread = std::thread(
        [&driver_node_ptr]() {
          driver_node_ptr->realtime_loop();
        });
    auto controller_realtime_thread = std::thread(
        [&controller_node_ptr]() {
          controller_node_ptr->realtime_loop();
        });

    // TODO(carlosvg): add wait loop or experiment duration option
    const std::chrono::seconds EXPERIMENT_DURATION = std::chrono::seconds(10);
    std::this_thread::sleep_for(EXPERIMENT_DURATION);

    rclcpp::shutdown();
    executor_thread.join();
    driver_realtime_thread.join();
    controller_realtime_thread.join();

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
