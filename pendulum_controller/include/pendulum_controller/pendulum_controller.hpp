#ifndef PENDULUM_CONTROLLER_H
#define PENDULUM_CONTROLLER_H


#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

namespace pendulum {

class PendulumController {
public:

    virtual void update_setpoint_data(const pendulum_msgs::msg::JointCommand &msg) = 0;

    virtual void update_sensor_data(const pendulum_msgs::msg::JointState &msg) = 0;

    virtual float compute_output() = 0;

};

} // namespace pendulum

#endif //PENDULUM_CONTROLLER_H
