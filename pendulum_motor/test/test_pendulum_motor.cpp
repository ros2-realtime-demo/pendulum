#include <iostream>
#include <chrono>
#include <memory>

#include "simple_pendulum_sim.hpp"
#include "pendulum_motor/pendulum_motor_node.hpp"

using namespace std::chrono_literals;
using namespace pendulum;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    std::chrono::nanoseconds update_period = 1000000ns;
    std::unique_ptr<PendulumMotor> motor =
            std::make_unique<PendulumMotorSim>(update_period);
    auto motor_node =  std::make_shared<PendulumMotorNode>(
            "pendulum_motor_node",
            update_period,
            std::move(motor),
            rclcpp::NodeOptions().use_intra_process_comms(true));

    exec.add_node(motor_node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
