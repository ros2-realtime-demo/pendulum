// Copyright 2019 Carlos San Vicente
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

#include <string>
#include <vector>

#include "pendulum_controller/pendulum_controller_node.hpp"

namespace pendulum
{
namespace pendulum_controller
{

PendulumControllerNode::PendulumControllerNode(const rclcpp::NodeOptions & options)
: PendulumControllerNode("pendulum_controller", options)
{}

PendulumControllerNode::PendulumControllerNode(
  const std::string & node_name,
  rclcpp::NodeOptions options)
: LifecycleNode(
    node_name,
    options),
  state_topic_name_(declare_parameter("state_topic_name").get<std::string>()),
  command_topic_name_(declare_parameter("command_topic_name").get<std::string>()),
  teleop_topic_name_(declare_parameter("teleop_topic_name").get<std::string>()),
  command_publish_period_(std::chrono::microseconds{
      declare_parameter("command_publish_period_us").get<std::uint16_t>()}),
  enable_topic_stats_(declare_parameter("enable_topic_stats").get<bool>()),
  topic_stats_topic_name_{declare_parameter("topic_stats_topic_name").get<std::string>()},
  topic_stats_publish_period_{std::chrono::milliseconds {
        declare_parameter("topic_stats_publish_period_ms").get<std::uint16_t>()}},
  deadline_duration_{std::chrono::milliseconds {
        declare_parameter("deadline_duration_ms").get<std::uint16_t>()}},
  controller_(PendulumController::Config(
      declare_parameter("controller.feedback_matrix").get<std::vector<double>>()))
{
  create_teleoperation_subscription();
  create_state_subscription();
  create_command_publisher();
  create_command_timer_callback();
}

void PendulumControllerNode::create_teleoperation_subscription()
{
  auto on_pendulum_teleop = [this](const pendulum2_msgs::msg::PendulumTeleop::SharedPtr msg) {
      controller_.set_teleop(msg->cart_position, msg->cart_velocity);
    };
  teleop_sub_ = this->create_subscription<pendulum2_msgs::msg::PendulumTeleop>(
    teleop_topic_name_, rclcpp::QoS(10), on_pendulum_teleop);
}

void PendulumControllerNode::create_state_subscription()
{
  rclcpp::SubscriptionOptions state_subscription_options;
  state_subscription_options.event_callbacks.deadline_callback =
    [](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      // do nothing for instrumenting purposes
      // in a real-application we may want to trigger an error for a specific deadline misses
    };
  if (enable_topic_stats_) {
    state_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    state_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
    state_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
  }
  auto on_sensor_message = [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      controller_.set_state(
        msg->position[0], msg->velocity[0],
        msg->position[1], msg->velocity[1]);
    };
  state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_name_,
    rclcpp::QoS(10).deadline(deadline_duration_),
    on_sensor_message,
    state_subscription_options);
}

void PendulumControllerNode::create_command_publisher()
{
  rclcpp::PublisherOptions command_publisher_options;
  command_publisher_options.event_callbacks.deadline_callback =
    [](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      // do nothing for instrumenting purposes
      // in a real-application we may want to trigger an error for a specific deadline misses
    };
  command_pub_ = this->create_publisher<pendulum2_msgs::msg::JointCommandStamped>(
    command_topic_name_,
    rclcpp::QoS(10).deadline(deadline_duration_),
    command_publisher_options);
}

void PendulumControllerNode::create_command_timer_callback()
{
  auto control_timer_callback = [this]() {
      controller_.update();
      command_message_.cmd.force = controller_.get_force_command();
      command_message_.header.stamp = this->get_clock()->now();
      command_pub_->publish(command_message_);
    };
  command_timer_ = this->create_wall_timer(command_publish_period_, control_timer_callback);
  // cancel immediately to prevent triggering it in this state
  command_timer_->cancel();
}

void PendulumControllerNode::log_controller_state()
{
  const auto state = controller_.get_state();
  const auto teleoperation_command = controller_.get_teleop();
  const double force_command = controller_.get_force_command();

  RCLCPP_INFO(get_logger(), "Cart position = %lf", state.at(0));
  RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.at(1));
  RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.at(2));
  RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.at(3));
  RCLCPP_INFO(get_logger(), "Teleoperation cart position = %lf", teleoperation_command.at(0));
  RCLCPP_INFO(get_logger(), "Teleoperation cart velocity = %lf", teleoperation_command.at(1));
  RCLCPP_INFO(get_logger(), "Force command = %lf", force_command);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  // reset internal state of the controller for a clean start
  controller_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  command_pub_->on_activate();
  command_timer_->reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  command_timer_->cancel();
  command_pub_->on_deactivate();
  // log the status to introspect the result
  log_controller_state();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace pendulum_controller
}  // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::pendulum_controller::PendulumControllerNode)
