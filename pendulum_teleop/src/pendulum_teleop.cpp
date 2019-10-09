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

#include "pendulum_teleop/pendulum_manager.hpp"
#include <memory>

void print_menu()
{
  std::cout << "MENU" << std::endl;
  std::cout << "1: Configure pendulum " << std::endl;
  std::cout << "2: Activate pendulum " << std::endl;
  std::cout << "3: Deactivate pendulum " << std::endl;
  std::cout << "4: Cleanup pendulum " << std::endl;
  std::cout << "Enter your choice : ";
}

int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  auto pendulum_manager = std::make_shared<pendulum::PendulumManager>(
    "pendulum_manager",
    "pendulum_controller_node",
    "pendulum_motor_node");
  exe.add_node(pendulum_manager);

  std::shared_future<void> activate_pendulum = std::async(std::launch::async,
      [&exe]() {exe.spin();});

  int choice = 0;
  do {
    print_menu();
    std::cin >> choice;
    switch (choice) {
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
      default:
        std::cout << "Invalid input" << std::endl;
    }
  } while (rclcpp::ok());

  rclcpp::shutdown();

  return 0;
}
