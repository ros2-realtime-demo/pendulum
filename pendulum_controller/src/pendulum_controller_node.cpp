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
#include <memory>

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "pendulum_controller/pendulum_controller_node.hpp"

namespace pendulum_controller
{
PendulumControllerNode::PendulumControllerNode(const rclcpp::NodeOptions & options)
: PendulumControllerNode("pendulum_controller", options)
{}

PendulumControllerNode::PendulumControllerNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options),
  state_topic_name_(declare_parameter<std::string>("state_topic_name", "pendulum_joint_states")),
  command_topic_name_(declare_parameter<std::string>("command_topic_name", "joint_command")),
  teleop_topic_name_(declare_parameter<std::string>("teleop_topic_name", "teleop")),
  deadline_duration_{std::chrono::milliseconds {
      declare_parameter<std::uint16_t>("deadline_us", 2000U)}},
  controller_(PendulumController::Config(
      declare_parameter<std::vector<double>>("controller.feedback_matrix",
      {-10.0000, -51.5393, 356.8637, 154.4146}))),
  num_missed_deadlines_{0U},
  realtime_cb_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false)),
  auto_start_node_(declare_parameter<bool>("auto_start_node", false)),
  proc_settings_(
    utils::ProcessSettings(
      declare_parameter<bool>("proc_settings.lock_memory", false),
      declare_parameter<std::uint16_t>("proc_settings.process_priority", 0U),
      declare_parameter<std::uint16_t>("proc_settings.cpu_affinity", 0U),
      declare_parameter<std::uint16_t>("proc_settings.lock_memory_size_mb", 0U),
      declare_parameter<bool>("proc_settings.configure_child_threads", false)
    )
  )
{
  create_teleoperation_subscription();
  create_state_subscription();
  create_command_publisher();
  create_wait_set();
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
  // Pre-allocates message in a pool
  using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  auto state_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum2_msgs::msg::JointState, 1>>();

  rclcpp::SubscriptionOptions state_subscription_options;
  state_subscription_options.callback_group = realtime_cb_group_;
  auto on_sensor_message = [this](const pendulum2_msgs::msg::JointState::SharedPtr msg) {
      update_controller(*msg);
      // publish pendulum force command
      command_message_.force = controller_.get_force_command();
      command_pub_->publish(command_message_);
    };
  state_sub_ = this->create_subscription<pendulum2_msgs::msg::JointState>(
    state_topic_name_,
    rclcpp::QoS(1),
    on_sensor_message,
    state_subscription_options,
    state_msg_strategy);
}

void PendulumControllerNode::create_wait_set()
{
  wait_set_ = std::make_shared<rclcpp::StaticWaitSet<1, 0, 0, 0, 0, 0>>(
    std::array<rclcpp::StaticWaitSet<1, 0, 0, 0, 0, 0>::SubscriptionEntry, 1>{{{state_sub_}}});
  // call wait to enforce initial allocations
  auto wait_result = wait_set_->wait(std::chrono::milliseconds(0));
  (void)wait_result;
}

void PendulumControllerNode::create_command_publisher()
{
  rclcpp::PublisherOptions command_publisher_options;
  command_publisher_options.callback_group = realtime_cb_group_;
  command_pub_ = this->create_publisher<pendulum2_msgs::msg::JointCommand>(
    command_topic_name_,
    rclcpp::QoS(1),
    command_publisher_options);
}

void PendulumControllerNode::start()
{
  if (auto_start_node_) {
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != this->configure().id()) {
      throw std::runtime_error("Could not configure " + std::string(this->get_name()));
    }
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != this->activate().id()) {
      throw std::runtime_error("Could not activate " + std::string(this->get_name()));
    }
  }
}

void PendulumControllerNode::run_realtime_loop()
{
  wait_for_driver();
  while (rclcpp::ok()) {
    update_realtime_loop();
  }
}

void PendulumControllerNode::wait_for_driver()
{
  bool is_ready = false;
  while (rclcpp::ok() && !is_ready) {
    bool sub_matched = command_pub_->get_subscription_count() == 1U;
    bool pub_matched = state_sub_->get_publisher_count() == 1U;
    bool is_active = is_active_.load();
    is_ready = sub_matched && pub_matched && is_active;
    if (!is_ready) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void PendulumControllerNode::update_realtime_loop()
{
  const auto wait_result = wait_set_->wait(deadline_duration_);
  if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
    if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
      pendulum2_msgs::msg::JointState msg;
      rclcpp::MessageInfo msg_info;
      if (state_sub_->take(msg, msg_info)) {
        update_controller(msg);
        // publish pendulum force command
        command_message_.force = controller_.get_force_command();
        command_pub_->publish(command_message_);
      } else {
        // msg not valid
      }
    }
  } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
    if (is_active_.load()) {
      ++num_missed_deadlines_;
    }
  }
}

void PendulumControllerNode::update_controller(const pendulum2_msgs::msg::JointState & msg)
{
  // update pendulum state
  controller_.set_state(
    msg.cart_position, msg.cart_velocity,
    msg.pole_angle, msg.pole_velocity);

  // update pendulum controller output
  controller_.update();
}

void PendulumControllerNode::log_controller_state()
{
  const auto state = controller_.get_state();
  const auto teleoperation_command = controller_.get_teleop();
  const double force_command = controller_.get_force_command();

  RCLCPP_INFO(get_logger(), "Cart position = %lf", state.cart_position);
  RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.cart_velocity);
  RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.pole_angle);
  RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.pole_velocity);
  RCLCPP_INFO(
    get_logger(), "Teleoperation cart position = %lf",
    teleoperation_command.cart_position);
  RCLCPP_INFO(
    get_logger(), "Teleoperation cart velocity = %lf",
    teleoperation_command.cart_velocity);
  RCLCPP_INFO(get_logger(), "Force command = %lf", force_command);
  RCLCPP_INFO(get_logger(), "Num missed deadlines = %u", num_missed_deadlines_);
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
  is_active_.store(true);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  command_pub_->on_deactivate();
  // log the status to introspect the result
  log_controller_state();
  is_active_.store(false);
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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum_controller::PendulumControllerNode)
