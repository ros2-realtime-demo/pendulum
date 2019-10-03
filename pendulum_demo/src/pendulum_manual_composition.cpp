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

#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <pendulum_ex_msgs/msg/pendulum_stats.hpp>

#include <iostream>
#include <memory>
#include <utility>

#ifdef PENDULUM_DEMO_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>
#endif

#ifdef PENDULUM_DEMO_TLSF_ENABLED
#include <tlsf_cpp/tlsf.hpp>
#endif

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"


#include "pendulum_motor_node/pendulum_motor_node.hpp"
#include "pendulum_motor_driver/pendulum_motor_driver.hpp"
#include "pendulum_motor_driver/simple_pendulum_sim.hpp"
#include "pendulum_controller_node/pendulum_controller_node.hpp"
#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_controller/pid_controller.hpp"

#ifdef PENDULUM_DEMO_TLSF_ENABLED
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;
#endif

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
static const char * OPTION_MEMORY_CHECK = "--memory-check";
static const char * OPTION_TLSF = "--use-tlsf";
static const char * OPTION_LOCK_MEMORY = "--lock-memory";
static const char * OPTION_PUBLISH_STATISTICS = "--pub-stats";

void print_usage()
{
  printf("Usage for pendulum_test:\n");
  printf("pendulum_test\n"
    "\t[%s pid proportional gain]\n"
    "\t[%s pid integral gain]\n"
    "\t[%s pid derivative gain]\n"
    "\t[%s controller update period]\n"
    "\t[%s physics simulation update period]\n"
    "\t[%s motor sensor update period]\n"
    "\t[%s use OSRF memory check tool]\n"
    "\t[%s lock memory]\n"
    "\t[%s publish statistics]\n"
    "\t[%s use TLSF allocator]\n"
    "\t[-h]\n",
    OPTION_PID_K,
    OPTION_PID_I,
    OPTION_PID_D,
    OPTION_CONTROLLER_UPDATE_PERIOD,
    OPTION_PHYSICS_UPDATE_PERIOD,
    OPTION_SENSOR_UPDATE_PERIOD,
    OPTION_MEMORY_CHECK,
    OPTION_LOCK_MEMORY,
    OPTION_PUBLISH_STATISTICS,
    OPTION_TLSF);
}

int main(int argc, char * argv[])
{
  bool use_memory_check = false;
  bool lock_memory = false;
  bool publish_statistics = false;
  bool use_tlfs = false;
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
    std::cout << "Enable memory check\n";
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LOCK_MEMORY)) {
    lock_memory = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_STATISTICS)) {
    publish_statistics = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_TLSF)) {
    use_tlfs = true;
  }

  // use a dummy period to initialize rttest
  struct timespec dummy_period;
  dummy_period.tv_sec = 0;
  dummy_period.tv_nsec = 1000000;
  rttest_init(1, dummy_period, SCHED_FIFO, 80, 0, NULL);
  rclcpp::init(argc, argv);

  // Initialize the executor.
  rclcpp::executor::ExecutorArgs exec_args;
  #ifdef PENDULUM_DEMO_TLSF_ENABLED
  // One of the arguments passed to the Executor is the memory strategy, which delegates the
  // runtime-execution allocations to the TLSF allocator.
  if (use_tlfs) {
    std::cout << "Enable TLSF allocator\n";
    rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
      std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
    exec_args.memory_strategy = memory_strategy;
  }
  #endif
  rclcpp::executors::SingleThreadedExecutor exec(exec_args);

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

  // Initialize the logger publisher.
  //if (publish_statistics){
    std::cout << "publish_statistics\n";
    auto node_stats = rclcpp::Node::make_shared("pendulum_statistics_node");
    auto controller_stats_pub =
     node_stats->create_publisher<pendulum_ex_msgs::msg::ControllerStats>(
      "controller_statistics", rclcpp::QoS(1));
    auto motor_stats_pub = node_stats->create_publisher<pendulum_ex_msgs::msg::MotorStats>(
      "motor_statistics", rclcpp::QoS(1));
    std::chrono::nanoseconds logger_publisher_period(100000000);
    // Create a lambda function that will fire regularly to publish the next results message.
    auto logger_publish_callback =
      [&controller_stats_pub, &motor_stats_pub, &motor_node, &controller_node]() {
        pendulum_ex_msgs::msg::ControllerStats controller_stats_msg;
        controller_node->update_sys_usage();
        controller_stats_msg = controller_node->get_controller_stats_message();
        controller_stats_pub->publish(controller_stats_msg);

        pendulum_ex_msgs::msg::MotorStats motor_stats_msg;
        motor_node->update_sys_usage();
        motor_stats_msg = motor_node->get_motor_stats_message();
        motor_stats_pub->publish(motor_stats_msg);
      };
    auto logger_publisher_timer = node_stats->create_wall_timer(
      logger_publisher_period, logger_publish_callback);
    exec.add_node(node_stats);
  //}

  // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
  // deterministic (real-time safe) algorithm, round robin.
  if (rttest_set_sched_priority(80, SCHED_RR)) {
    perror("Couldn't set scheduling priority and policy");
  }

  // Lock the currently cached virtual memory into RAM, as well as any future memory allocations,
  // and do our best to prefault the locked memory to prevent future pagefaults.
  // Will return with a non-zero error code if something went wrong (insufficient resources or
  // permissions).
  // Always do this as the last step of the initialization phase.
  // See README.md for instructions on setting permissions.
  // See rttest/rttest.cpp for more details.
  if (lock_memory) {
    std::cout << "lock memory on\n";
    if (rttest_lock_and_prefault_dynamic() != 0) {
      fprintf(stderr, "Couldn't lock all cached virtual memory.\n");
      fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM will be recorded.\n");
    }
  }

  exec.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
