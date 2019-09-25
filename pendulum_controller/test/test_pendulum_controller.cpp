// Copyright 2019
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

#include "rttest/rttest.h"
#include "rttest/utils.h"

#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pendulum_controller/pid_controller.hpp"

using namespace std::chrono_literals;
using namespace pendulum;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    PIDProperties pid;
    pid.p = 1.5;
    pid.i = 0.0;
    std::chrono::nanoseconds update_period = 970000ns;
    std::unique_ptr<PendulumController> pid_controller =
            std::make_unique<PIDController>(update_period, pid);
    auto controller_node =  std::make_shared<PendulumControllerNode>(
            "pendulum_controller",
            std::move(pid_controller),
            update_period,
            rclcpp::NodeOptions().use_intra_process_comms(true));

    exec.add_node(controller_node->get_node_base_interface());

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
