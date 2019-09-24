#ifndef PENDULUM_MOTOR_H
#define PENDULUM_MOTOR_H

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

namespace pendulum {

class PendulumMotor {
public:

    virtual void update_motor_command(const pendulum_msgs::msg::JointCommand &msg) = 0;

    virtual void update_motor_state() = 0;

    virtual float get_position() const = 0;
    virtual float get_velocity() const = 0;
};

} // namespace pendulum

#endif //PENDULUM_MOTOR_H
