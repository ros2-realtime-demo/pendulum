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
#include <memory>

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "pendulum_driver/pendulum_driver_node.hpp"

namespace pendulum
{
namespace pendulum_driver
{
PendulumDriverNode::PendulumDriverNode(const rclcpp::NodeOptions & options)
: PendulumDriverNode("pendulum_driver", options)
{}

PendulumDriverNode::PendulumDriverNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options),
  state_topic_name_(declare_parameter("state_topic_name").get<std::string>()),
  command_topic_name_(declare_parameter("command_topic_name").get<std::string>()),
  disturbance_topic_name_(declare_parameter("disturbance_topic_name").get<std::string>()),
  cart_base_joint_name_(declare_parameter("cart_base_joint_name").get<std::string>()),
  pole_joint_name_(declare_parameter("pole_joint_name").get<std::string>()),
  state_publish_period_(std::chrono::microseconds{
      declare_parameter("state_publish_period_us").get<std::uint16_t>()}),
  enable_topic_stats_(declare_parameter("enable_topic_stats").get<bool>()),
  topic_stats_topic_name_{declare_parameter("topic_stats_topic_name").get<std::string>()},
  topic_stats_publish_period_{std::chrono::milliseconds{
        declare_parameter("topic_stats_publish_period_ms").get<std::uint16_t>()}},
  deadline_duration_{std::chrono::milliseconds{
        declare_parameter("deadline_duration_ms").get<std::uint16_t>()}},
  driver_(
    PendulumDriver::Config(
      declare_parameter("driver.pendulum_mass").get<double>(),
      declare_parameter("driver.cart_mass").get<double>(),
      declare_parameter("driver.pendulum_length").get<double>(),
      declare_parameter("driver.damping_coefficient").get<double>(),
      declare_parameter("driver.gravity").get<double>(),
      declare_parameter("driver.max_cart_force").get<double>(),
      declare_parameter("driver.noise_level").get<double>(),
      std::chrono::microseconds {state_publish_period_}
    )
  ),
  num_missed_deadlines_pub_{0U},
  num_missed_deadlines_sub_{0U}
{
  init_state_message();
  create_state_publisher();
  create_command_subscription();
  create_disturbance_subscription();
  create_state_timer_callback();
}

void PendulumDriverNode::init_state_message()
{
  state_message_.cart_position = 0.0;
  state_message_.cart_velocity = 0.0;
  state_message_.cart_force = 0.0;
  state_message_.pole_angle = 0.0;
  state_message_.pole_velocity = 0.0;
}

void PendulumDriverNode::create_state_publisher()
{
  rclcpp::PublisherOptions sensor_publisher_options;
  sensor_publisher_options.event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
    {
      num_missed_deadlines_pub_++;
    };
  state_pub_ = this->create_publisher<pendulum2_msgs::msg::JointState>(
    state_topic_name_,
    rclcpp::QoS(10).deadline(deadline_duration_),
    sensor_publisher_options);
}

void PendulumDriverNode::create_command_subscription()
{
  // Pre-allocates message in a pool
  using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum2_msgs::msg::JointCommand, 1>>();

  rclcpp::SubscriptionOptions command_subscription_options;
  command_subscription_options.event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
    {
      num_missed_deadlines_sub_++;
    };
  if (enable_topic_stats_) {
    command_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    command_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
    command_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
  }
  auto on_command_received = [this](pendulum2_msgs::msg::JointCommand::SharedPtr msg) {
      driver_.set_controller_cart_force(msg->force);
    };
  command_sub_ = this->create_subscription<pendulum2_msgs::msg::JointCommand>(
    command_topic_name_,
    rclcpp::QoS(10).deadline(deadline_duration_),
    on_command_received,
    command_subscription_options,
    command_msg_strategy);
}

void PendulumDriverNode::create_disturbance_subscription()
{
  auto on_disturbance_received = [this](pendulum2_msgs::msg::JointCommand::SharedPtr msg) {
      driver_.set_disturbance_force(msg->force);
    };
  disturbance_sub_ = this->create_subscription<pendulum2_msgs::msg::JointCommand>(
    disturbance_topic_name_, rclcpp::QoS(10), on_disturbance_received);
}

void PendulumDriverNode::create_state_timer_callback()
{
  auto state_timer_callback = [this]() {
      driver_.update();
      const auto state = driver_.get_state();
      state_message_.cart_position = state.cart_position;
      state_message_.cart_velocity = state.cart_velocity;
      state_message_.cart_force = state.cart_force;
      state_message_.pole_angle = state.pole_angle;
      state_message_.pole_velocity = state.pole_velocity;
      state_pub_->publish(state_message_);
    };
  state_timer_ = this->create_wall_timer(state_publish_period_, state_timer_callback);
  // cancel immediately to prevent triggering it in this state
  state_timer_->cancel();
}

void PendulumDriverNode::log_driver_state()
{
  const auto state = driver_.get_state();
  const auto disturbance_force = driver_.get_disturbance_force();
  const double controller_force_command = driver_.get_controller_cart_force();

  RCLCPP_INFO(get_logger(), "Cart position = %lf", state.cart_position);
  RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.cart_velocity);
  RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.pole_angle);
  RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.pole_velocity);
  RCLCPP_INFO(get_logger(), "Controller force command = %lf", controller_force_command);
  RCLCPP_INFO(get_logger(), "Disturbance force = %lf", disturbance_force);
  RCLCPP_INFO(get_logger(), "Publisher missed deadlines = %lu", num_missed_deadlines_pub_);
  RCLCPP_INFO(get_logger(), "Subscription missed deadlines = %lu", num_missed_deadlines_sub_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  // reset internal state of the driver for a clean start
  driver_.reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  state_pub_->on_activate();
  state_timer_->reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  state_timer_->cancel();
  state_pub_->on_deactivate();
  // log the status to introspect the result
  log_driver_state();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace pendulum_driver
}  // namespace pendulum

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum::pendulum_driver::PendulumDriverNode)
