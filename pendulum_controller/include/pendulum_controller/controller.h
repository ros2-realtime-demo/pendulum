#ifndef PENDULUM_CONTROLLER_H
#define PENDULUM_CONTROLLER_H


#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

namespace pendulum {

class Controller {
public:
    virtual void update_setpoint_data(const pendulum_msgs::msg::JointCommand &msg);

    virtual void update_sensor_data(const pendulum_msgs::msg::JointState &msg);

    virtual float get_controller_input() const;

    virtual float get_controller_output() const;

    virtual float compute_output();

private:
    std::atomic<float> controller_input;
    std::atomic<float> controller_output;
};

} // namespace pendulum

#endif //PENDULUM_CONTROLLER_H
