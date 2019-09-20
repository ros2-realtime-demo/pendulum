#include "pendulum_controller/controller.h"
#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_controller/PIDController.hpp"


#include <iostream>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace pendulum;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    PIDProperties pid;

    std::unique_ptr<Controller> pid_controller =
            std::make_unique<PIDController>(std::chrono::nanoseconds(960000), pid);

    auto controller_node =  std::make_unique<ControllerNode>(
            "pendulum_controller",
            rclcpp::NodeOptions().use_intra_process_comms(true),
            std::move(pid_controller));
    
    return EXIT_SUCCESS;
}