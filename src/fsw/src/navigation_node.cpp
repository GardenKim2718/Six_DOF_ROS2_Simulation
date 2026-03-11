/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      navigation_node.cpp
 * @brief     6-DOF navigation node source file
 *
 * @date      2026-03-05 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-11 updated by Chungwon Kim due to addition of actuators
 */

#include "fsw/navigation_node.hpp"

Navigation::Navigation()
: Node("navigation_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Navigation node...");
    
    //QoS settings
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Declare Parameters
    this->declare_parameter<double>("loop_rate_hz", loop_rate_hz_);

    // Get parameters
    GetParameters();

    RCLCPP_INFO(this->get_logger(),
        "Navigation Node Parameters: loop_rate_hz=%.3f", loop_rate_hz_);

    // Subscribers Initialization
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state", qos_profile,
        std::bind(&Navigation::CallbackState, this, std::placeholders::_1));
    
    // Publishers Initialization
    pub_navigation_ = this->create_publisher<interfaces::msg::Navigation>(
        "navigation", qos_profile);
    
    // Steady clock initialization
    steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Timer Initialization
    rclcpp::Time current_time = steady_clock_->now();

    // Run Navigation Loop
    const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / loop_rate_hz_));

    t_run_node_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        steady_clock_,
        period_ns,
        std::bind(&Navigation::Run, this)
    );
}

Navigation::~Navigation()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Navigation node...");
}

void Navigation::GetParameters()
{
    // fetch parameters and store them in member variables
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
}

void Navigation::Init()
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Navigation Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);
}

void Navigation::Run()
{
    // handle initialization
    if (!b_simulator_initialized_) {
        RCLCPP_WARN(this->get_logger(),
            "Waiting for simulator initialization...");
        return;
    }

    if (!b_navigation_initialized_) {
        Init();
        b_navigation_initialized_ = true;
    }

    // get subscribed state
    interfaces::msg::State current_state;
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        current_state = last_state_;
    }

    // publish navigation message
    // temporarily, just pass the state message as navigation message without any processing
    o_navigation_.id = current_state.id;
    o_navigation_.header = current_state.header;
    o_navigation_.pose = current_state.pose;
    o_navigation_.vel = current_state.vel;
    o_navigation_.rwa_momentum = current_state.rwa_momentum;
    pub_navigation_->publish(o_navigation_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation>());

  rclcpp::shutdown();
  return 0;
}