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

#ifdef PENDULUM_DEMO_MEMORYTOOLS_ENABLED
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>
#endif

#ifdef PENDULUM_DEMO_TLSF_ENABLED
#include <tlsf_cpp/tlsf.hpp>
#endif

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_driver/pendulum_driver_interface.hpp"
#include "pendulum_simulation/pendulum_simulation.hpp"
#include "pendulum_tools/memory_lock.hpp"
#include "pendulum_tools/rt_thread.hpp"

#ifdef PENDULUM_DEMO_TLSF_ENABLED
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;
#endif

static const size_t DEFAULT_DEADLINE_PERIOD_NS = 2000000;
static const int DEFAULT_PRIORITY = 0;
static const size_t DEFAULT_STATISTICS_PERIOD_MS = 1000;

static const char * OPTION_MEMORY_CHECK = "--memory-check";
static const char * OPTION_TLSF = "--use-tlsf";
static const char * OPTION_LOCK_MEMORY = "--lock-memory";
static const char * OPTION_PRIORITY = "--priority";
static const char * OPTION_CPU_AFFINITY = "--cpu-affinity";
static const char * OPTION_PUBLISH_STATISTICS = "--pub-stats";
static const char * OPTION_DEADLINE_PERIOD = "--deadline";
static const char * OPTION_STATISTICS_PERIOD = "--stats-period";

static const size_t DEFAULT_PHYSICS_UPDATE_PERIOD_NS = 10000000;
static const size_t DEFAULT_SENSOR_UPDATE_PERIOD_NS = 960000;

static const char * OPTION_SENSOR_UPDATE_PERIOD = "--sensor-period";
static const char * OPTION_PHYSICS_UPDATE_PERIOD = "--physics-period";

void print_usage(std::string program_name)
{
  printf("Usage for %s:\n", program_name.c_str());
  printf("%s\n"
    "\t[%s physics simulation update period (ns)]\n"
    "\t[%s pendulum sensor update period (ns)]\n"
    "\t[%s deadline QoS period (ms)]\n"
    "\t[%s use OSRF memory check tool]\n"
    "\t[%s lock memory]\n"
    "\t[%s set process real-time priority]\n"
    "\t[%s set process cpu affinity]\n"
    "\t[%s statistics publisher period (ms)]\n"
    "\t[%s publish statistics (enable)]\n"
    "\t[%s use TLSF allocator]\n"
    "\t[-h]\n",
    program_name.c_str(),
    OPTION_PHYSICS_UPDATE_PERIOD,
    OPTION_SENSOR_UPDATE_PERIOD,
    OPTION_DEADLINE_PERIOD,
    OPTION_MEMORY_CHECK,
    OPTION_LOCK_MEMORY,
    OPTION_PRIORITY,
    OPTION_CPU_AFFINITY,
    OPTION_STATISTICS_PERIOD,
    OPTION_PUBLISH_STATISTICS,
    OPTION_TLSF);
}

int main(int argc, char * argv[])
{
  // common options
  bool use_memory_check = false;
  bool lock_memory = false;
  bool publish_statistics = false;
  bool use_tlfs = false;
  int process_priority = DEFAULT_PRIORITY;
  uint32_t cpu_affinity = 0;
  std::chrono::nanoseconds deadline_duration(DEFAULT_DEADLINE_PERIOD_NS);
  std::chrono::milliseconds logger_publisher_period(DEFAULT_STATISTICS_PERIOD_MS);

  // driver options
  std::chrono::nanoseconds sensor_publish_period(DEFAULT_SENSOR_UPDATE_PERIOD_NS);
  std::chrono::nanoseconds physics_update_period(DEFAULT_PHYSICS_UPDATE_PERIOD_NS);

  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string prog_name(argv[0]);
  prog_name = prog_name.substr(prog_name.find_last_of("/\\")+1);

  // Argument count and usage
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage(prog_name);
    return 0;
  }

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_MEMORY_CHECK)) {
    use_memory_check = true;
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
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PRIORITY)) {
    process_priority = std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_PRIORITY));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CPU_AFFINITY)) {
    cpu_affinity = std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_CPU_AFFINITY));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_DEADLINE_PERIOD)) {
    deadline_duration = std::chrono::nanoseconds(
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_DEADLINE_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_STATISTICS_PERIOD)) {
    logger_publisher_period = std::chrono::milliseconds(
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_STATISTICS_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_SENSOR_UPDATE_PERIOD)) {
    sensor_publish_period = std::chrono::nanoseconds(
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_SENSOR_UPDATE_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PHYSICS_UPDATE_PERIOD)) {
    physics_update_period = std::chrono::nanoseconds(
      std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_PHYSICS_UPDATE_PERIOD)));
  }

  // TODO(carlossvg): check options
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

  // set QoS deadline period
  rclcpp::QoS qos_deadline_profile(10);
  qos_deadline_profile.deadline(deadline_duration);

  // Create pendulum simulation
  std::unique_ptr<pendulum::PendulumDriverInterface> sim =
    std::make_unique<pendulum::PendulumSimulation>(physics_update_period);

  // Create pendulum driver node
  pendulum::PendulumDriverOptions driver_options;
  driver_options.node_name = "pendulum_driver";
  driver_options.status_publish_period = sensor_publish_period;
  driver_options.status_qos_profile = qos_deadline_profile;
  driver_options.enable_statistics = publish_statistics;
  driver_options.statistics_publish_period = logger_publisher_period;
  driver_options.enable_check_memory = use_memory_check;

  auto pendulum_driver = std::make_shared<pendulum::PendulumDriverNode>(
    std::move(sim),
    driver_options,
    rclcpp::NodeOptions().use_intra_process_comms(true));
  exec.add_node(pendulum_driver->get_node_base_interface());

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

  // Lock the currently cached virtual memory into RAM, as well as any future memory allocations,
  // and do our best to prefault the locked memory to prevent future pagefaults.
  // Will return with a non-zero error code if something went wrong (insufficient resources or
  // permissions).
  if (lock_memory) {
    std::cout << "Enable lock memory\n";
    if (pendulum::lock_and_prefault_dynamic() != 0) {
      fprintf(stderr, "Couldn't lock all cached virtual memory.\n");
      fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM will be recorded.\n");
    }
  }

  exec.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
