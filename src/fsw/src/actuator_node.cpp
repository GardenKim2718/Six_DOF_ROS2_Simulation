/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      actuator_node.cpp
 * @brief     6-DOF actuator node source file
 *
 * @date      2026-03-05 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#include "fsw/actuator_node.hpp"

Actuator::Actuator()
: Node("actuator_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Actuator node...");
    
    //QoS settings
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Declare Parameters
    this->declare_parameter("loop_rate_hz", loop_rate_hz_);

    // Get parameters
    GetParameters();

    RCLCPP_INFO(this->get_logger(),
        "Control Node Parameters: loop_rate_hz=%.3f", loop_rate_hz_);

    // Subscribers Initialization
    sub_command_ = this->create_subscription<interfaces::msg::Command>(
        "command", qos_profile,
        std::bind(&Actuator::CallbackCommand, this, std::placeholders::_1));

    // Publishers Initialization
    pub_actuator_ = this->create_publisher<interfaces::msg::Actuator>(
        "actuator", qos_profile);

    // Steady clock initialization
    steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Run Contol Loop
    const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / loop_rate_hz_));

    t_run_node_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        steady_clock_,
        period_ns,
        std::bind(&Actuator::Run, this)
    );
}

Actuator::~Actuator()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Actuator node...");
}

void Actuator::GetParameters()
{
    // fetch parameters and store them in member variables
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
}

void Actuator::Init()
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Actuator Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);
}

void Actuator::Run()
{
    // handle initialization
    if (!b_control_initialized_) {
        RCLCPP_WARN(this->get_logger(),
            "Waiting for control initialization...");
        return;
    }

    // get subscribed command
    interfaces::msg::Command current_command;
    {
        std::lock_guard<std::mutex> lock(mutex_command_);
        current_command = last_command_;
    }

    // compute actuator command from control command
    // temporarily, just pass the Command message as Actuator message without processing
    o_actuator_.id = current_command.id;
    o_actuator_.force = current_command.force;
    o_actuator_.torque = current_command.torque;
    pub_actuator_->publish(o_actuator_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Actuator>());

  rclcpp::shutdown();
  return 0;
}