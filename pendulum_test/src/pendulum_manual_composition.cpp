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

#include "rclcpp/rclcpp.hpp"
#include "simple_pendulum_sim.hpp"
#include "pid_controller.hpp"
#include "pendulum_motor/pendulum_motor.hpp"
#include "pendulum_motor/pendulum_motor_node.hpp"
#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"

using namespace std::chrono_literals;
using namespace pendulum;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    PIDProperties pid;
    pid.p = 1.0;

    std::chrono::nanoseconds update_period = 960000ns;
    std::unique_ptr<PendulumController> pid_controller =
            std::make_unique<PIDController>(update_period, pid);
    auto controller_node =  std::make_shared<PendulumControllerNode>(
            "pendulum_controller",
            std::move(pid_controller),
            update_period,
            rclcpp::NodeOptions().use_intra_process_comms(true));



    std::chrono::nanoseconds controller_publish_period =  800000ns;
    std::chrono::nanoseconds sensor_publish_period =  800000ns;
    std::chrono::nanoseconds physics_update_period = 1000000ns;
    std::unique_ptr<PendulumMotor> motor =
            std::make_unique<PendulumMotorSim>(controller_publish_period);
    auto motor_node =  std::make_shared<PendulumMotorNode>(
            "pendulum_motor_node",
            sensor_publish_period,
            physics_update_period,
            std::move(motor),
            rclcpp::NodeOptions().use_intra_process_comms(true));

    exec.add_node(controller_node->get_node_base_interface());
    exec.add_node(motor_node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
