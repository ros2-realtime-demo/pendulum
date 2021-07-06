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

namespace pendulum_driver
{
PendulumDriverNode::PendulumDriverNode(const rclcpp::NodeOptions & options)
: PendulumDriverNode("pendulum_driver", options)
{}

PendulumDriverNode::PendulumDriverNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options),
  state_topic_name_(declare_parameter<std::string>("state_topic_name", "pendulum_joint_states")),
  command_topic_name_(declare_parameter<std::string>("command_topic_name", "joint_command")),
  disturbance_topic_name_(declare_parameter<std::string>("disturbance_topic_name", "disturbance")),
  cart_base_joint_name_(declare_parameter<std::string>("cart_base_joint_name", "cart_base_joint")),
  pole_joint_name_(declare_parameter<std::string>("pole_joint_name", "pole_joint")),
  update_period_(std::chrono::microseconds{
    declare_parameter<std::uint16_t>("update_period_us", 1000U)}),
  driver_(
    PendulumDriver::Config(
      declare_parameter<double>("driver.pendulum_mass", 1.0),
      declare_parameter<double>("driver.cart_mass", 5.0),
      declare_parameter<double>("driver.pendulum_length", 2.0),
      declare_parameter<double>("driver.damping_coefficient", 20.0),
      declare_parameter<double>("driver.gravity", -9.8),
      declare_parameter<double>("driver.max_cart_force", 1000.0),
      declare_parameter<double>("driver.noise_level", 1.0),
      std::chrono::microseconds {update_period_}
    )
  ),
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
  ),
  num_missed_deadlines_{0U},
  num_missed_messages_{0U}
{
  init_state_message();
  create_state_publisher();
  create_command_subscription();
  create_disturbance_subscription();
  create_state_timer_callback();
  create_wait_set();
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
  sensor_publisher_options.callback_group = realtime_cb_group_;
  state_pub_ = this->create_publisher<pendulum2_msgs::msg::JointState>(
    state_topic_name_,
    rclcpp::QoS(1),
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
  command_subscription_options.callback_group = realtime_cb_group_;
  auto on_command_received = [this](pendulum2_msgs::msg::JointCommand::SharedPtr msg) {
      driver_.set_controller_cart_force(msg->force);
    };
  command_sub_ = this->create_subscription<pendulum2_msgs::msg::JointCommand>(
    command_topic_name_,
    rclcpp::QoS(1),
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
  auto state_timer_callback = [this]() -> void {
      driver_.update();
      const auto state = driver_.get_state();
      state_message_.cart_position = state.cart_position;
      state_message_.cart_velocity = state.cart_velocity;
      state_message_.cart_force = state.cart_force;
      state_message_.pole_angle = state.pole_angle;
      state_message_.pole_velocity = state.pole_velocity;
      state_pub_->publish(state_message_);
    };
  state_timer_ =
    this->create_wall_timer(update_period_, state_timer_callback, realtime_cb_group_);
  // cancel immediately to prevent triggering it in this state
  state_timer_->cancel();
}

void PendulumDriverNode::create_wait_set()
{
  wait_set_ = std::make_shared<rclcpp::StaticWaitSet<0, 0, 1, 0, 0, 0>>(
    std::array<rclcpp::StaticWaitSet<0, 0, 1, 0, 0, 0>::SubscriptionEntry, 0>{},
    std::array<rclcpp::GuardCondition::SharedPtr, 0>{},
    std::array<rclcpp::TimerBase::SharedPtr, 1>{state_timer_});
  // call wait to enforce initial allocations
  auto wait_result = wait_set_->wait(std::chrono::milliseconds(0));
  (void)wait_result;
}

void PendulumDriverNode::start()
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

void PendulumDriverNode::wait_for_start(bool wait_for_publisher_matched)
{
  bool is_ready = false;
  bool sub_matched = false;
  while (rclcpp::ok() && !is_ready) {
    if (wait_for_publisher_matched) {
      sub_matched = state_pub_->get_subscription_count() == 1U;
    }
    bool is_active = is_active_.load();
    is_ready = wait_for_publisher_matched ? (sub_matched && is_active) : is_active;
    if (!is_ready) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void PendulumDriverNode::run_realtime_loop()
{
  wait_for_start();
  utils::periodic_task_init(period_info_, update_period_);
  while (rclcpp::ok()) {
    wait_rest_of_period(period_info_);

    auto timepoint_before_task = std::chrono::steady_clock::now();
    update_realtime_loop();
    auto timepoint_after_task = std::chrono::steady_clock::now();

    // update period and overruns
    auto overruns = increment_period_until_no_overruns(period_info_);
    num_missed_deadlines_ += overruns;

    update_rt_loop_statistics(timepoint_before_task, timepoint_after_task);
  }
}

void PendulumDriverNode::update_realtime_loop()
{
  pendulum2_msgs::msg::JointCommand msg;
  rclcpp::MessageInfo msg_info;
  // take a msg if it is available
  if (command_sub_->take(msg, msg_info)) {
    driver_.set_controller_cart_force(msg.force);
  } else {
    ++num_missed_messages_;
  }
  state_timer_->execute_callback();
  ++iteration_;
}

void PendulumDriverNode::update_rt_loop_statistics(
  std::chrono::steady_clock::time_point timepoint_before_task,
  std::chrono::steady_clock::time_point timepoint_after_task)
{
  auto wake_up_latency = period_info_.wake_up_time - timepoint_before_task;
  auto task_duration = timepoint_after_task - timepoint_before_task;
  {
    RealtimeStatisticsTracker::ScopedAccess<ThreadType::realtime> tr(wake_up_latency_tracker_);
    tr->add_sample(std::chrono::duration_cast<std::chrono::microseconds>(wake_up_latency).count());
    tr->add_sample(wake_up_latency.count());
  }
  {
    RealtimeStatisticsTracker::ScopedAccess<ThreadType::realtime> tr(rt_task_duration_tracker_);
    tr->add_sample(std::chrono::duration_cast<std::chrono::microseconds>(task_duration).count());
  }
}

void PendulumDriverNode::log_driver_state()
{
  const auto state = driver_.get_state();
  const auto disturbance_force = driver_.get_disturbance_force();
  const double controller_force_command = driver_.get_controller_cart_force();

  RCLCPP_INFO(get_logger(), "Iteration %lu", iteration_.load());
  RCLCPP_INFO(get_logger(), "Cart position = %lf", state.cart_position);
  RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.cart_velocity);
  RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.pole_angle);
  RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.pole_velocity);
  RCLCPP_INFO(get_logger(), "Controller force command = %lf", controller_force_command);
  RCLCPP_INFO(get_logger(), "Disturbance force = %lf", disturbance_force);
  {
    RealtimeStatisticsTracker::ScopedAccess<ThreadType::nonRealtime> tr(wake_up_latency_tracker_);
    RCLCPP_INFO(
      get_logger(), "Timer wake-up latency (us): avg=%lf, min=%lf, max=%lf, std=%lf",
      tr->mean(), tr->min(), tr->max(), std::sqrt(tr->variance()));
  }
  {
    RealtimeStatisticsTracker::ScopedAccess<ThreadType::nonRealtime> tr(rt_task_duration_tracker_);
    RCLCPP_INFO(
      get_logger(), "Real-time task duration (us): avg=%lf, min=%lf, max=%lf, std=%lf",
      tr->mean(), tr->min(), tr->max(), std::sqrt(tr->variance()));
  }
  RCLCPP_INFO(get_logger(), "Num missed deadlines = %lu", num_missed_deadlines_.load());
  RCLCPP_INFO(get_logger(), "Num missed messages = %lu", num_missed_messages_.load());
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
  is_active_.store(true);
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
  is_active_.store(false);
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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pendulum_driver::PendulumDriverNode)
