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
#include <string>
#include "rcutils/cmdline_parser.h"
#include "pendulum_teleop/pendulum_manager.hpp"

void print_menu()
{
  std::cout << std::endl;
  std::cout << "Menu" << std::endl;
  std::cout << "-----------------------" << std::endl;
  std::cout << "0: Exit " << std::endl;
  std::cout << "1: Configure pendulum " << std::endl;
  std::cout << "2: Activate pendulum " << std::endl;
  std::cout << "3: Deactivate pendulum " << std::endl;
  std::cout << "4: Cleanup pendulum " << std::endl;
  std::cout << "5: Shutdown pendulum " << std::endl;
  std::cout << "Enter your choice : ";
}

int main(int argc, char ** argv)
{
  std::string manager_node_name = "pendulum_manager";
  std::string controller_node_name = "pendulum_controller";
  std::string motor_node_name = "pendulum_driver";

  if (rcutils_cli_option_exist(argv, argv + argc, "--manager-name")) {
    manager_node_name = rcutils_cli_get_option(argv, argv + argc, "--manager-name");
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "controller-name")) {
    controller_node_name = rcutils_cli_get_option(argv, argv + argc, "--controller-name");
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "--motor-name")) {
    motor_node_name = rcutils_cli_get_option(argv, argv + argc, "--motor-name");
  }
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  auto pendulum_manager = std::make_shared<pendulum::PendulumManager>(
    manager_node_name,
    controller_node_name,
    motor_node_name);
  exe.add_node(pendulum_manager);

  std::shared_future<void> activate_pendulum = std::async(std::launch::async,
      [&exe]() {exe.spin();});

  int choice = 0;
  do {
    print_menu();
    std::cin >> choice;
    switch (choice) {
      case 0:
        std::cout << "Exit" << std::endl;
        break;
      case 1:
        pendulum_manager->configure_controller();
        pendulum_manager->configure_motor();
        break;
      case 2:
        pendulum_manager->activate_controller();
        pendulum_manager->activate_motor();
        break;
      case 3:
        pendulum_manager->deactivate_controller();
        pendulum_manager->deactivate_motor();
        break;
      case 4:
        pendulum_manager->cleanup_controller();
        pendulum_manager->cleanup_motor();
        break;
      case 5:
        pendulum_manager->shutdown_controller();
        pendulum_manager->shutdown_motor();
        break;
      default:
        std::cout << "Invalid input" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (rclcpp::ok() && choice != 0);

  rclcpp::shutdown();

  return 0;
}
