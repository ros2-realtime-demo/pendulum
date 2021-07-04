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

#include <gtest/gtest.h>
#include <vector>
#include <memory>

#include "apex_test_tools/apex_test_tools.hpp"

#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"

using pendulum::pendulum_driver::PendulumDriverNode;
using pendulum::pendulum_controller::PendulumControllerNode;
using pendulum::utils::ProcessSettings;
using pendulum::utils::configure_process_priority;
using pendulum::utils::lock_process_memory;

class TestPendulum : public ::testing::Test
{
protected:
  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  std::shared_ptr<PendulumDriverNode> driver_node_ptr;
  std::shared_ptr<PendulumControllerNode> controller_node_ptr;
  // ProcessSettings controller_settings;
  // ProcessSettings driver_settings;

  static void SetUpTestCase()
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());
  }

  void SetUp() override
  {
    params.emplace_back("state_topic_name", "joint_states");
    params.emplace_back("command_topic_name", "command");
    params.emplace_back("disturbance_topic_name", "disturbance");
    params.emplace_back("cart_base_joint_name", "cart_base_joint");
    params.emplace_back("pole_joint_name", "pole_joint");
    params.emplace_back("update_period_us", 1000);
    params.emplace_back("deadline_us", 2000);
    params.emplace_back("driver.pendulum_mass", 1.0);
    params.emplace_back("driver.cart_mass", 5.0);
    params.emplace_back("driver.pendulum_length", 2.0);
    params.emplace_back("driver.damping_coefficient", 20.0);
    params.emplace_back("driver.gravity", -9.8);
    params.emplace_back("driver.max_cart_force", 1000.0);
    params.emplace_back("driver.noise_level", 1.0);
    node_options.parameter_overrides(params);

    driver_node_ptr = std::make_shared<PendulumDriverNode>("driver");
    controller_node_ptr = std::make_shared<PendulumControllerNode>("controller");
    // controller_settings = controller_node_ptr->get_proc_settings();
    // driver_settings = driver_node_ptr->get_proc_settings();
  }

  static void TearDownTestCase()
  {
    (void)rclcpp::shutdown();
  }
};


TEST_F(TestPendulum, real_time_loop_does_not_allocate)
{
  // Create a static executor to run non-real time tasks
  rclcpp::executors::StaticSingleThreadedExecutor exec;
  exec.add_node(driver_node_ptr->get_node_base_interface());
  exec.add_node(controller_node_ptr->get_node_base_interface());

  auto controller_settings = controller_node_ptr->get_proc_settings();
  auto driver_settings = driver_node_ptr->get_proc_settings();

  auto thread = std::thread([&exec]() {exec.spin();});

  // Create a thread to run real-time tasks
  auto driver_thread = std::thread(
    [this, driver_settings]() {
      configure_process_priority(driver_settings.process_priority, driver_settings.cpu_affinity);
      apex_test_tools::memory_test::start();
      driver_node_ptr->run_realtime_loop();
      apex_test_tools::memory_test::stop();
    });
  auto controller_thread = std::thread(
    [this, controller_settings]() {
      configure_process_priority(
        controller_settings.process_priority,
        controller_settings.cpu_affinity);
      apex_test_tools::memory_test::start();
      controller_node_ptr->run_realtime_loop();
      apex_test_tools::memory_test::stop();
    });

  driver_node_ptr->start();
  controller_node_ptr->start();

  rclcpp::sleep_for(std::chrono::seconds(10));

  rclcpp::shutdown();
  thread.join();
  driver_thread.join();
  controller_thread.join();
}
