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

#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "pendulum_motor_node/pendulum_motor_node.hpp"
#include "pendulum_motor_driver/pendulum_motor_driver.hpp"
#include "pendulum_motor_driver/simple_pendulum_sim.hpp"
#include "pendulum_controller_node/pendulum_controller_node.hpp"
#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_controller/pid_controller.hpp"

static const double DEFAULT_PID_K = 1.0;
static const double DEFAULT_PID_I = 0.0;
static const double DEFAULT_PID_D = 0.0;

static const char * OPTION_PID_K = "--pid-k";
static const char * OPTION_PID_I = "--pid-i";
static const char * OPTION_PID_D = "--pid-d";

static const size_t DEFAULT_CONTROLLER_UPDATE_PERIOD = 970000;
static const size_t DEFAULT_PHYSICS_UPDATE_PERIOD = 10000000;
static const size_t DEFAULT_SENSOR_UPDATE_PERIOD = 960000;

static const char * OPTION_CONTROLLER_UPDATE_PERIOD = "--controller-update";
static const char * OPTION_PHYSICS_UPDATE_PERIOD = "--physics-update";
static const char * OPTION_SENSOR_UPDATE_PERIOD = "--sensor-update";
static const char * OPTION_MEMORY_CHECK = "--memory_check";


void print_usage()
{
  printf("Usage for pendulum_test:\n");
  printf("pendulum_test "
    "[%s pid proportional gain] "
    "[%s pid integral gain] "
    "[%s pid derivative gain] "
    "[%s controller update period] "
    "[%s physics simulation update period] "
    "[%s motor sensor update period] "
    "[%s use OSRF memory check tool] "
    "[-h]\n",
    OPTION_PID_K,
    OPTION_PID_I,
    OPTION_PID_D,
    OPTION_CONTROLLER_UPDATE_PERIOD,
    OPTION_PHYSICS_UPDATE_PERIOD,
    OPTION_SENSOR_UPDATE_PERIOD,
    OPTION_MEMORY_CHECK);
}

int main(int argc, char * argv[])
{
  bool use_memory_check = false;
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Argument count and usage
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }


  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_MEMORY_CHECK)) {
    use_memory_check = true;
  }

  // use a dummy period to initialize rttest
  struct timespec dummy_period;
  dummy_period.tv_sec = 0;
  dummy_period.tv_nsec = 1000000;
  rttest_init(1, dummy_period, SCHED_FIFO, 80, 0, NULL);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::chrono::milliseconds deadline_duration(10);
  rclcpp::QoS qos_deadline_profile(10);
  qos_deadline_profile.deadline(deadline_duration);

  pendulum::PIDProperties pid;
  pid.p = 0;
  pid.i = 0;
  std::chrono::nanoseconds update_period = std::chrono::nanoseconds(970000);
  std::unique_ptr<pendulum::PendulumController> pid_controller =
    std::make_unique<pendulum::PIDController>(update_period, pid);
  auto controller_node = std::make_shared<pendulum::PendulumControllerNode>(
    "pendulum_controller",
    std::move(pid_controller),
    update_period,
    qos_deadline_profile,
    rclcpp::QoS(1),
    use_memory_check,
    rclcpp::NodeOptions().use_intra_process_comms(true));
  exec.add_node(controller_node->get_node_base_interface());

  std::chrono::nanoseconds sensor_publish_period = std::chrono::nanoseconds(960000);
  std::chrono::nanoseconds physics_update_period = std::chrono::nanoseconds(1000000);


  std::unique_ptr<pendulum::PendulumMotor> motor =
    std::make_unique<pendulum::PendulumMotorSim>(physics_update_period);
  auto motor_node = std::make_shared<pendulum::PendulumMotorNode>(
    "pendulum_motor_node",
    std::move(motor),
    sensor_publish_period,
    qos_deadline_profile,
    use_memory_check,
    rclcpp::NodeOptions().use_intra_process_comms(true));
  exec.add_node(motor_node->get_node_base_interface());

  // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
  // deterministic (real-time safe) algorithm, round robin.
  if (rttest_set_sched_priority(98, SCHED_RR)) {
    perror("Couldn't set scheduling priority and policy");
  }

  // Lock the currently cached virtual memory into RAM, as well as any future memory allocations,
  // and do our best to prefault the locked memory to prevent future pagefaults.
  // Will return with a non-zero error code if something went wrong (insufficient resources or
  // permissions).
  // Always do this as the last step of the initialization phase.
  // See README.md for instructions on setting permissions.
  // See rttest/rttest.cpp for more details.
  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory.\n");
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM will be recorded.\n");
  }

  exec.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
