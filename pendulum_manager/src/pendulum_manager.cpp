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
#include "pendulum_manager/pendulum_node_manager.hpp"

void print_menu()
{
  std::cout << std::endl;
  std::cout << "Menu options" << std::endl;
  std::cout << "-----------------------------------------";
  std::cout << "-----------------------------------------" << std::endl;

  std::cout << " q: Exit " << std::endl;

  std::cout << " 0: Pendulum:   Configure and activate" << std::endl;
  std::cout << " 1: Pendulum:   Configure  " << " | ";
  std::cout << " w: Controller: Configure  " << " | ";
  std::cout << " a: Driver:     Configure  " << std::endl;

  std::cout << " 2: Pendulum:   Activate   " << " | ";
  std::cout << " e: Controller: Activate   " << " | ";
  std::cout << " s: Driver:     Activate   " << std::endl;

  std::cout << " 3: Pendulum:   Deactivate " << " | ";
  std::cout << " r: Controller: Deactivate " << " | ";
  std::cout << " d: Driver:     Deactivate " << std::endl;

  std::cout << " 4: Pendulum:   Cleanup    " << " | ";
  std::cout << " t: Controller: Cleanup    " << " | ";
  std::cout << " f: Driver:     Cleanup    " << std::endl;

  std::cout << " 5: Pendulum:   Shutdown   " << " | ";
  std::cout << " y: Controller: Shutdown   " << " | ";
  std::cout << " g: Driver:     Shutdown   " << std::endl;

  std::cout << "Enter your choice : ";
}

int main(int argc, char ** argv)
{
  std::string manager_node_name = "pendulum_manager";
  std::string controller_node_name = "pendulum_controller";
  std::string driver_node_name = "pendulum_driver";

  if (rcutils_cli_option_exist(argv, argv + argc, "--manager-name")) {
    manager_node_name = rcutils_cli_get_option(argv, argv + argc, "--manager-name");
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "controller-name")) {
    controller_node_name = rcutils_cli_get_option(argv, argv + argc, "--controller-name");
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "--driver-name")) {
    driver_node_name = rcutils_cli_get_option(argv, argv + argc, "--driver-name");
  }

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  auto pendulum_manager = std::make_shared<pendulum::PendulumNodeManager>(
    manager_node_name,
    controller_node_name,
    driver_node_name);
  exe.add_node(pendulum_manager);

  std::shared_future<void> activate_pendulum = std::async(
    std::launch::async,
    [&exe]() {exe.spin();});

  char choice = 0;
  do {
    print_menu();
    std::cin >> choice;
    switch (choice) {
      case 'q':
        std::cout << "Exit" << std::endl;
        break;
      case '0':
        pendulum_manager->configure_controller();
        pendulum_manager->configure_driver();
        pendulum_manager->activate_controller();
        pendulum_manager->activate_driver();
        break;
      case '1':
        pendulum_manager->configure_controller();
        pendulum_manager->configure_driver();
        break;
      case '2':
        pendulum_manager->activate_controller();
        pendulum_manager->activate_driver();
        break;
      case '3':
        pendulum_manager->deactivate_controller();
        pendulum_manager->deactivate_driver();
        break;
      case '4':
        pendulum_manager->cleanup_controller();
        pendulum_manager->cleanup_driver();
        break;
      case '5':
        pendulum_manager->shutdown_controller();
        pendulum_manager->shutdown_driver();
        break;
      case 'w':
        pendulum_manager->configure_controller();
        break;
      case 'e':
        pendulum_manager->activate_controller();
        break;
      case 'r':
        pendulum_manager->deactivate_controller();
        break;
      case 't':
        pendulum_manager->cleanup_controller();
        break;
      case 'y':
        pendulum_manager->shutdown_controller();
        break;
      case 'a':
        pendulum_manager->configure_driver();
        break;
      case 's':
        pendulum_manager->activate_driver();
        break;
      case 'd':
        pendulum_manager->deactivate_driver();
        break;
      case 'f':
        pendulum_manager->cleanup_driver();
        break;
      case 'g':
        pendulum_manager->shutdown_driver();
        break;
      default:
        std::cout << "Invalid input" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (rclcpp::ok() && choice != 'q');

  rclcpp::shutdown();

  return 0;
}
