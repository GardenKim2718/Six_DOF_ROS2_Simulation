/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      control_node.cpp
 * @brief     6-DOF control node source file
 *
 * @date      2026-02-09 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-02-13 expanded by Chungwon Kim for 6-DOF control
 */

#include "fsw/control_node.hpp"

Control::Control()
: Node("control_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Control node...");
    
    //QoS settings
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Declare Parameters
    this->declare_parameter("loop_rate_hz", loop_rate_hz_);
    this->declare_parameter("linear_kp", linear_kp_);
    this->declare_parameter("linear_kd", linear_kd_);
    this->declare_parameter("linear_ki", linear_ki_);
    this->declare_parameter("angular_kp", angular_kp_);
    this->declare_parameter("angular_kd", angular_kd_);
    this->declare_parameter("angular_ki", angular_ki_);

    // Get parameters
    GetParameters();

    RCLCPP_INFO(this->get_logger(),
        "Control Node Parameters: loop_rate_hz=%.3f", loop_rate_hz_);
    
    RCLCPP_INFO(this->get_logger(),
        "Linear control Gains: linear_kp=%.3f, linear_kd=%.3f, linear_ki=%.3f",
        linear_kp_, linear_kd_, linear_ki_);
    
    RCLCPP_INFO(this->get_logger(),
        "Angular control Gains: angular_kp=%.3f, angular_kd=%.3f, angular_ki=%.3f",
        angular_kp_, angular_kd_, angular_ki_);

    // Subscribers Initialization
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state", qos_profile,
        std::bind(&Control::CallbackState, this, std::placeholders::_1));

    sub_guidance_ = this->create_subscription<interfaces::msg::Guidance>(
        "guidance", qos_profile,
        std::bind(&Control::CallbackGuidance, this, std::placeholders::_1));

    // Publishers Initialization
    pub_command_ = this->create_publisher<interfaces::msg::Command>(
        "command", qos_profile);
    
    // Timer Initialization
    rclcpp::Time current_time = steady_clock.now();

    // Wait for simulator and guidance initialization
    while (rclcpp::ok() && (!b_simulator_initialized_ || !b_guidance_initialized_))
    {
        RCLCPP_INFO(this->get_logger(), *get_clock(), 1000,
                    "Waiting for simulator and guidance initialization...");
    }

    Init();

    // Run Contol Loop
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });
}

Control::~Control()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Control node...");
}

void Control::GetParameters()
{
    // This function can be used to fetch parameters when needed
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
    this->get_parameter("linear_kp", linear_kp_);
    this->get_parameter("linear_kd", linear_kd_);
    this->get_parameter("linear_ki", linear_ki_);
    this->get_parameter("angular_kp", angular_kp_);
    this->get_parameter("angular_kd", angular_kd_);
    this->get_parameter("angular_ki", angular_ki_);
}

void Control::Init()
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Control Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);

    // Initialize time
    sim_time_prev_ = last_state_.header.stamp;
    real_time_prev_ = steady_clock.now();

    // Initialize command message
    o_command_.id = last_state_.id;
    o_command_.force.x = 0.0;
    o_command_.force.y = 0.0;
    o_command_.force.z = 0.0;
    o_command_.torque.x = 0.0;
    o_command_.torque.y = 0.0;
    o_command_.torque.z = 0.0;
}

void Control::Run()
{
    // time
    auto current_time = steady_clock.now();

    // time interval
    time_dt_ = (current_time - real_time_prev_).seconds();
    if (time_dt_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "Non-positive time step detected: dt=%.6f. Skipping control update.", time_dt_);
        return;
    }
    else {
        sim_time_curr_ = sim_time_prev_ + rclcpp::Duration::from_seconds(time_dt_);
    }
    real_time_prev_ = current_time;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());

  rclcpp::shutdown();
  return 0;
}