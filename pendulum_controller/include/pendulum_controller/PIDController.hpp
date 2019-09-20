#include <chrono>
#include <cmath>
#include <atomic>

#include "controller.h"

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum {

/// Struct for storing PID controller properties.
struct PIDProperties
{
    /// Proportional constant.
    double p = 1;
    /// Integral constant.
    double i = 0;
    /// Derivative constant.
    double d = 0;
    /// Desired state of the plant.
    double command = PI / 2;
};


class PIDController : public Controller
{
public:

    PIDController(std::chrono::nanoseconds period, PIDProperties pid)
    : publish_period_(period), pid_(pid)
    {
        // Calculate the controller timestep (for discrete differentiation/integration).
        dt_ = publish_period_.count() / (1000.0 * 1000.0 * 1000.0);
        if (std::isnan(dt_) || dt_ == 0) {
            throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
        }
    }

    virtual void update_setpoint_data(const pendulum_msgs::msg::JointCommand &msg) override
    {
        setpoint_position_ = msg.position;
    }

    virtual void update_sensor_data(const pendulum_msgs::msg::JointState &msg) override
    {
        sensor_position_ = msg.position;
    }

    virtual float compute_output() override
    {
        float command_position_output = 0;
        if (std::isnan(sensor_position_)) {
            throw std::runtime_error("Sensor value was NaN in on_sensor_message callback");
        }
        // PID controller algorithm
        double error = pid_.command - sensor_position_;
        // Proportional gain is proportional to error
        double p_gain = pid_.p * error;
        // Integral gain is proportional to the accumulation of error
        i_gain_ = pid_.i * (i_gain_ + error * dt_);
        // Differential gain is proportional to the change in error
        double d_gain = pid_.d * (error - last_error_) / dt_;
        last_error_ = error;

        // Calculate the message based on PID gains
        command_position_output = sensor_position_ + p_gain + i_gain_ + d_gain;
        // Enforce positional limits
        if (command_position_output > PI) {
            command_position_output = PI;
        } else if (command_position_output < 0) {
            command_position_output = 0;
        }

        if (std::isnan(command_position_output)) {
            throw std::runtime_error("Resulting command was NaN in on_sensor_message callback");
        }
        return command_position_output;
    }

private:
    std::chrono::nanoseconds publish_period_;
    PIDProperties pid_;
    double last_error_ = 0;
    double i_gain_ = 0;
    double dt_;
    std::atomic<float> setpoint_position_;
    std::atomic<float> sensor_position_;
};


} // namespace pendulum