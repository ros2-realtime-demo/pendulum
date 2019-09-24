#include "pendulum_controller/pendulum_controller.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pid_controller.hpp"


#include <iostream>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace pendulum;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    PIDProperties pid;
    pid.p = 2.0;
    rclcpp::executors::SingleThreadedExecutor exec;

    std::chrono::nanoseconds update_period = 1000000ns;
    std::unique_ptr<PendulumController> pid_controller =
            std::make_unique<PIDController>(update_period, pid);
    auto controller_node =  std::make_shared<PendulumControllerNode>(
            "pendulum_controller",
            std::move(pid_controller),
            update_period,
            rclcpp::NodeOptions().use_intra_process_comms(true));

    exec.add_node(controller_node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
