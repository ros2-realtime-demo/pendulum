// Copyright 2020 Carlos San Vicente
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

#ifndef PENDULUM_DEMO_PENDULUM_DEMO_SETTINGS_HPP
#define PENDULUM_DEMO_PENDULUM_DEMO_SETTINGS_HPP

#include "rcutils/cmdline_parser.h"
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "pendulum_tools/memory_lock.hpp"
#include "pendulum_tools/rt_thread.hpp"

struct DemoSettings
{
  void print_usage()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("settings"),
      "\t[%s auto activate nodes]\n"
      "\t[%s lock memory]\n"
      "\t[%s lock a fixed memory size in MB]\n"
      "\t[%s set process real-time priority]\n"
      "\t[%s set process cpu affinity]\n"
      "\t[-h]\n",
      OPTION_AUTO_ACTIVATE_NODES.c_str(),
      OPTION_LOCK_MEMORY.c_str(),
      OPTION_LOCK_MEMORY_SIZE.c_str(),
      OPTION_PRIORITY.c_str(),
      OPTION_CPU_AFFINITY.c_str());
  }

  bool init(int argc, char * argv[])
  {
    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
      print_usage();
      return false;
    }
    // Optional argument parsing
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_AUTO_ACTIVATE_NODES.c_str())) {
      auto_activate = true;
    }
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LOCK_MEMORY.c_str())) {
      lock_memory = true;
    }
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LOCK_MEMORY_SIZE.c_str())) {
      lock_memory = true;
      lock_memory_size_mb =
        std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_LOCK_MEMORY_SIZE.c_str()));
    }
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PRIORITY.c_str())) {
      process_priority = std::stoi(
        rcutils_cli_get_option(
          argv, argv + argc,
          OPTION_PRIORITY.c_str()));
    }
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_CPU_AFFINITY.c_str())) {
      cpu_affinity = std::stoi(
        rcutils_cli_get_option(
          argv, argv + argc,
          OPTION_CPU_AFFINITY.c_str()));
    }
    return true;
  }

  void configure_process()
  {
    // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
    // deterministic (real-time safe) algorithm, fifo.
    if (process_priority > 0 && process_priority < 99) {
      if (pendulum::set_this_thread_priority(process_priority, SCHED_FIFO)) {
        throw std::runtime_error("Couldn't set scheduling priority and policy");
      }
    }
    if (cpu_affinity > 0U) {
      if (pendulum::set_this_thread_cpu_affinity(cpu_affinity)) {
        throw std::runtime_error("Couldn't set cpu affinity");
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
        throw std::runtime_error("Couldn't lock  virtual memory");
      }
    }
  }

  const std::string OPTION_AUTO_ACTIVATE_NODES = "--auto";
  const std::string OPTION_LOCK_MEMORY = "--lock-memory";
  const std::string OPTION_LOCK_MEMORY_SIZE = "--lock-memory-size";
  const std::string OPTION_PRIORITY = "--priority";
  const std::string OPTION_CPU_AFFINITY = "--cpu-affinity";

  /// automatically activate lifecycle nodes
  bool auto_activate = false;
  /// lock and prefault memory
  bool lock_memory = false;
  /// process priority value to set
  int process_priority = 0;
  /// process cpu affinity value to set
  uint32_t cpu_affinity = 0;
  /// Memory size to lock in Megabytes
  size_t lock_memory_size_mb = 0;
};

#endif //PENDULUM_DEMO_PENDULUM_DEMO_SETTINGS_HPP
