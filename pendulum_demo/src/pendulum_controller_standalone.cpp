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
#include <pendulum_msgs_v2/msg/pendulum_stats.hpp>

#include <vector>
#include <iostream>
#include <memory>
#include <utility>
#include <string>

#ifdef PENDULUM_DEMO_TLSF_ENABLED
#include <tlsf_cpp/tlsf.hpp>
#endif

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "pendulum_controller_node/pendulum_controller_node.hpp"
#include "pendulum_controller_node/pendulum_controller.hpp"
#include "pendulum_controllers/full_state_feedback_controller.hpp"
#include "pendulum_tools/memory_lock.hpp"
#include "pendulum_tools/rt_thread.hpp"

#ifdef PENDULUM_DEMO_TLSF_ENABLED
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;
#endif

static const size_t DEFAULT_DEADLINE_PERIOD_US = 2000;
static const int DEFAULT_PRIORITY = 0;
static const size_t DEFAULT_STATISTICS_PERIOD_MS = 100;

static const char * OPTION_AUTO_ACTIVATE_NODES = "--auto";
static const char * OPTION_TLSF = "--use-tlsf";
static const char * OPTION_LOCK_MEMORY = "--lock-memory";
static const char * OPTION_LOCK_MEMORY_SIZE = "--lock-memory-size";
static const char * OPTION_PRIORITY = "--priority";
static const char * OPTION_CPU_AFFINITY = "--cpu-affinity";
static const char * OPTION_PUBLISH_STATISTICS = "--pub-stats";
static const char * OPTION_DEADLINE_PERIOD = "--deadline";
static const char * OPTION_STATISTICS_PERIOD = "--stats-period";

static const size_t DEFAULT_CONTROLLER_UPDATE_PERIOD_US = 1000;
static const char * OPTION_CONTROLLER_UPDATE_PERIOD = "--controller-period";

static const char * OPTION_CONTROLLER_K1 = "--K1";
static const char * OPTION_CONTROLLER_K2 = "--K2";
static const char * OPTION_CONTROLLER_K3 = "--K3";
static const char * OPTION_CONTROLLER_K4 = "--K4";

void print_usage(std::string program_name)
{
  printf("Usage for %s:\n", program_name.c_str());
  printf("%s\n"
    "\t[%s auto activate nodes]\n"
    "\t[%s controller update period (ns)]\n"
    "\t[%s deadline QoS period (ms)]\n"
    "\t[%s lock memory]\n"
    "\t[%s lock a fixed memory size in MB]\n"
    "\t[%s set process real-time priority]\n"
    "\t[%s set process cpu affinity]\n"
    "\t[%s statistics publisher period (ms)]\n"
    "\t[%s publish statistics (enable)]\n"
    "\t[%s use TLSF allocator]\n"
    "\t[%s set feedback matrix K1]\n"
    "\t[%s set feedback matrix K2]\n"
    "\t[%s set feedback matrix K3]\n"
    "\t[%s set feedback matrix K4]\n"
    "\t[-h]\n",
    program_name.c_str(),
    OPTION_AUTO_ACTIVATE_NODES,
    OPTION_CONTROLLER_UPDATE_PERIOD,
    OPTION_DEADLINE_PERIOD,
    OPTION_LOCK_MEMORY,
    OPTION_LOCK_MEMORY_SIZE,
    OPTION_PRIORITY,
    OPTION_CPU_AFFINITY,
    OPTION_STATISTICS_PERIOD,
    OPTION_PUBLISH_STATISTICS,
    OPTION_TLSF,
    OPTION_CONTROLLER_K1,
    OPTION_CONTROLLER_K2,
    OPTION_CONTROLLER_K3,
    OPTION_CONTROLLER_K4);
}

int main(int argc, char * argv[])
{
  // common options
  bool auto_activate = false;
  bool lock_memory = false;
  bool publish_statistics = false;
  bool use_tlfs = false;
  int process_priority = DEFAULT_PRIORITY;
  uint32_t cpu_affinity = 0;
  size_t lock_memory_size_mb = 0;
  std::chrono::microseconds deadline_duration(DEFAULT_DEADLINE_PERIOD_US);
  std::chrono::milliseconds logger_publisher_period(DEFAULT_STATISTICS_PERIOD_MS);

  // controller options
  std::chrono::microseconds controller_update_period(DEFAULT_CONTROLLER_UPDATE_PERIOD_US);
  std::vector<double> feedback_matrix = {-10.0000, -51.5393, 356.8637, 154.4146};

  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string prog_name(argv[0]);
  prog_name = prog_name.substr(prog_name.find_last_of("/\\") + 1);

  // Argument count and usage
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage(prog_name);
    return 0;
  }

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_AUTO_ACTIVATE_NODES)) {
    auto_activate = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LOCK_MEMORY)) {
    lock_memory = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LOCK_MEMORY_SIZE)) {
    lock_memory = true;
    lock_memory_size_mb =
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_LOCK_MEMORY_SIZE));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_STATISTICS)) {
    publish_statistics = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_TLSF)) {
    use_tlfs = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PRIORITY)) {
    process_priority = std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_PRIORITY));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CPU_AFFINITY)) {
    cpu_affinity = std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_CPU_AFFINITY));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_DEADLINE_PERIOD)) {
    deadline_duration = std::chrono::microseconds(
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_DEADLINE_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_STATISTICS_PERIOD)) {
    logger_publisher_period = std::chrono::milliseconds(
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_STATISTICS_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CONTROLLER_UPDATE_PERIOD)) {
    controller_update_period = std::chrono::microseconds(
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_CONTROLLER_UPDATE_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CONTROLLER_K1)) {
    feedback_matrix[0] =
      std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_CONTROLLER_K1));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CONTROLLER_K2)) {
    feedback_matrix[1] =
      std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_CONTROLLER_K2));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CONTROLLER_K3)) {
    feedback_matrix[2] =
      std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_CONTROLLER_K3));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CONTROLLER_K4)) {
    feedback_matrix[3] =
      std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_CONTROLLER_K4));
  }

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

  // Use rmw_qos_profile_default with history size 10 and set QoS deadline period.
  // NOTE: rmw_qos_profile_default uses reliable policy by default
  rclcpp::QoS qos_deadline_profile(10);
  qos_deadline_profile.deadline(deadline_duration);

  // Create a controller
  std::unique_ptr<pendulum::PendulumController> controller = std::make_unique<
    pendulum::FullStateFeedbackController>(feedback_matrix);

  // Create pendulum controller node
  pendulum::PendulumControllerOptions controller_options;
  controller_options.node_name = "pendulum_controller";
  controller_options.command_publish_period = controller_update_period;
  controller_options.status_qos_profile = qos_deadline_profile;
  controller_options.command_qos_profile = qos_deadline_profile;
  controller_options.setpoint_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  controller_options.enable_statistics = publish_statistics;
  controller_options.statistics_publish_period = logger_publisher_period;

  auto controller_node = std::make_shared<pendulum::PendulumControllerNode>(
    std::move(controller),
    controller_options,
    rclcpp::NodeOptions().use_intra_process_comms(true));
  exec.add_node(controller_node->get_node_base_interface());

  // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
  // deterministic (real-time safe) algorithm, fifo.
  if (process_priority > 0 && process_priority < 99) {
    if (pendulum::set_this_thread_priority(process_priority, SCHED_FIFO)) {
      perror("Couldn't set scheduling priority and policy");
    }
  }
  if (cpu_affinity > 0U) {
    if (pendulum::set_this_thread_cpu_affinity(cpu_affinity)) {
      perror("Couldn't set cpu affinity");
    }
  }

  if (lock_memory) {
    int res = 0;
    if (lock_memory_size_mb > 0) {
      res = pendulum::lock_and_prefault_dynamic(lock_memory_size_mb * 1024 * 1024);
    } else {
      res = pendulum::lock_and_prefault_dynamic();
    }
    if (res != 0) {
      fprintf(stderr, "Couldn't lock  virtual memory.\n");
    } else {
      std::cout << "Memory locked succesfully\n";
    }
  }

  if (auto_activate) {
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != controller_node->configure().id()) {
      throw std::runtime_error("Could not configure PendulumControllerNode!");
    }
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != controller_node->activate().id()) {
      throw std::runtime_error("Could not activate PendulumControllerNode!");
    }
  }

  exec.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
