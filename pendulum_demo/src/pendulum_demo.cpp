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

#include "pendulum_demo/pendulum_demo_settings.hpp"

#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <string>

#ifdef PENDULUM_DEMO_TLSF_ENABLED
#include <tlsf_cpp/tlsf.hpp>
#endif

#include "rclcpp/rclcpp.hpp"

#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pendulum_tools/memory_lock.hpp"
#include "pendulum_tools/rt_thread.hpp"

#ifdef PENDULUM_DEMO_TLSF_ENABLED
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;
#endif

int main(int argc, char * argv[])
{
  DemoSettings settings;
  if(!settings.init(argc, argv)){
    return EXIT_FAILURE;
  }

  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);

    // Initialize the executor.
    rclcpp::ExecutorOptions exec_options;
#ifdef PENDULUM_DEMO_TLSF_ENABLED
    // One of the arguments passed to the Executor is the memory strategy, which delegates the
    // runtime-execution allocations to the TLSF allocator.
    if (settings.use_tlfs) {
      rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
        std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
      exec_options.memory_strategy = memory_strategy;
    }
#endif
    rclcpp::executors::SingleThreadedExecutor exec(exec_options);

    // Create pendulum controller node
    using pendulum::pendulum_controller::PendulumControllerNode;
    const auto controller_node_ptr = std::make_shared<PendulumControllerNode>
        ("pendulum_controller");

    exec.add_node(controller_node_ptr->get_node_base_interface());

    // Create pendulum simulation
    using pendulum::pendulum_driver::PendulumDriverNode;
    const auto driver_node_ptr = std::make_shared<PendulumDriverNode>
        ("pendulum_driver");

    exec.add_node(driver_node_ptr->get_node_base_interface());

    // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
    // deterministic (real-time safe) algorithm, fifo.
    if (settings.process_priority > 0 && settings.process_priority < 99) {
      if (pendulum::set_this_thread_priority(settings.process_priority, SCHED_FIFO)) {
        throw std::runtime_error("Couldn't set scheduling priority and policy");
      }
    }
    if (settings.cpu_affinity > 0U) {
      if (pendulum::set_this_thread_cpu_affinity(settings.cpu_affinity)) {
        throw std::runtime_error("Couldn't set cpu affinity");
      }
    }

    if (settings.lock_memory) {
      int res = 0;
      if (settings.lock_memory_size_mb > 0) {
        res = pendulum::lock_and_prefault_dynamic(settings.lock_memory_size_mb * 1024 * 1024);
      } else {
        res = pendulum::lock_and_prefault_dynamic();
      }
      if (res != 0) {
        throw std::runtime_error("Couldn't lock  virtual memory");
      }
    }

    if (settings.auto_activate) {
      if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE !=
          controller_node_ptr->configure().id()) {
        throw std::runtime_error("Could not configure PendulumControllerNode!");
      }
      if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE !=
          controller_node_ptr->activate().id()) {
        throw std::runtime_error("Could not activate PendulumControllerNode!");
      }
      if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE !=
          driver_node_ptr->configure().id()) {
        throw std::runtime_error("Could not configure PendulumDriverNode!");
      }
      if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != driver_node_ptr->activate().id()) {
        throw std::runtime_error("Could not activate PendulumDriverNode!");
      }
    }

    exec.spin();
    if (!rclcpp::shutdown()) {
      throw std::runtime_error("rclcpp shutdown failed!");
    }
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), "Unknown exception caught. "
                                                                "Exiting...");
    ret = -1;
  }
  return ret;
}
