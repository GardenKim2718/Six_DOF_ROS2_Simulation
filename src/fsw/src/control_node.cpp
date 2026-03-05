/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      control_node.cpp
 * @brief     6-DOF control node source file
 *
 * @date      2026-02-09 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-02-13 expanded by Chungwon Kim for 6-DOF control
 *            2026-03-03 edited by Chungwon Kim for Guidance-Command interface update
 *            2026-03-05 updated to use steady clock instead of wall timer
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

    this->declare_parameter("mass", mass_);

    this->declare_parameter("inertia_xx", 1.0);
    this->declare_parameter("inertia_yy", 1.0);
    this->declare_parameter("inertia_zz", 1.0);
    this->declare_parameter("inertia_xy", 0.0);
    this->declare_parameter("inertia_xz", 0.0);
    this->declare_parameter("inertia_yz", 0.0);

    this->declare_parameter("max_force", max_force_);
    this->declare_parameter("max_torque", max_torque_);

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

    sub_target_ = this->create_subscription<interfaces::msg::Target>(
        "target", qos_profile,
        std::bind(&Control::CallbackTarget, this, std::placeholders::_1));

    // Publishers Initialization
    pub_command_ = this->create_publisher<interfaces::msg::Command>(
        "command", qos_profile);

    // Steady clock initialization
    steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Timer Initialization
    rclcpp::Time current_time = steady_clock_->now();

    // Run Contol Loop
    const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / loop_rate_hz_));

    t_run_node_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        steady_clock_,
        period_ns,
        std::bind(&Control::Run, this)
    );
}

Control::~Control()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Control node...");
}

void Control::GetParameters()
{
    // fetch parameters and store them in member variables
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
    this->get_parameter("linear_kp", linear_kp_);
    this->get_parameter("linear_kd", linear_kd_);
    this->get_parameter("linear_ki", linear_ki_);
    this->get_parameter("angular_kp", angular_kp_);
    this->get_parameter("angular_kd", angular_kd_);
    this->get_parameter("angular_ki", angular_ki_);

    this->get_parameter("mass", mass_);

    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    this->get_parameter("inertia_xx", Ixx);
    this->get_parameter("inertia_yy", Iyy);
    this->get_parameter("inertia_zz", Izz);
    this->get_parameter("inertia_xy", Ixy);
    this->get_parameter("inertia_xz", Ixz);
    this->get_parameter("inertia_yz", Iyz);

    this->get_parameter("max_force", max_force_);
    this->get_parameter("max_torque", max_torque_);

    // fill in the MOI matrix
    inertia_ << Ixx, Ixy, Ixz,
                Ixy, Iyy, Iyz,
                Ixz, Iyz, Izz;
    
    // compute inverse MOI matrix
    inertia_inv_ = inertia_.inverse();
}

void Control::Init(const interfaces::msg::State& initial_state)
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Control Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);

    // Initialize time
    sim_time_prev_ = initial_state.header.stamp;

    // Initialize command message
    o_command_.id = initial_state.id;
    o_command_.force.x = 0.0;
    o_command_.force.y = 0.0;
    o_command_.force.z = 0.0;
    o_command_.torque.x = 0.0;
    o_command_.torque.y = 0.0;
    o_command_.torque.z = 0.0;
}

void Control::Run()
{
    // handle initialization
    if (!b_simulator_initialized_ || !b_guidance_initialized_ || !b_target_initialized_) {
        RCLCPP_WARN(this->get_logger(),
            "Waiting for simulator and guidance initialization...");
        return;
    }

    if (!b_control_initialized_) {
        Init(last_state_);
        b_control_initialized_ = true;
    }

    // get subscribed state
    interfaces::msg::State current_state;
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        current_state = last_state_;
        sim_time_curr_ = current_state.header.stamp;
    }

    // get subscribed guidance
    interfaces::msg::Guidance current_guidance;
    {
        std::lock_guard<std::mutex> lock(mutex_guidance_);
        current_guidance = last_guidance_;
    }

    // get subscribed target
    interfaces::msg::Target current_target;
    {
        std::lock_guard<std::mutex> lock(mutex_target_);
        current_target = last_target_;
    }

    // compute time step
    time_dt_ = (sim_time_curr_ - sim_time_prev_).seconds();
    sim_time_prev_ = sim_time_curr_;

    // check for guidance mode
    b_linear_guidance_active_ = current_guidance.linear_guidance_active;
    b_angular_guidance_active_ = current_guidance.angular_guidance_active;

    //-----------------------Attitude Control-------------------------//
    Eigen::Quaterniond q_current(
        current_state.pose.orientation.w,
        current_state.pose.orientation.x,
        current_state.pose.orientation.y,
        current_state.pose.orientation.z);
    q_current.normalize();

    Eigen::Vector3d w_current(
        current_state.vel.angular.x,
        current_state.vel.angular.y,
        current_state.vel.angular.z);

    Eigen::Vector3d torque_command;

    if (b_angular_guidance_active_)
    {
        Eigen::Vector3d angular_acc_command = Eigen::Vector3d(
            current_guidance.accel.angular.x,
            current_guidance.accel.angular.y,
            current_guidance.accel.angular.z);
        
        // convert angular acceleration command to torque command
        torque_command = inertia_ * angular_acc_command +
                         w_current.cross(inertia_ * w_current);
    } else 
    {
        // Use PID control for attitude control near target (APDG inactive)
        Eigen::Quaterniond q_desired(
            current_target.pose.orientation.w,
            current_target.pose.orientation.x,
            current_target.pose.orientation.y,
            current_target.pose.orientation.z);
        q_desired.normalize();

        Eigen::Vector3d w_desired(
            current_target.vel.angular.x,
            current_target.vel.angular.y,
            current_target.vel.angular.z);

        err_quat_ = QuaternionSignCorrection(q_desired * QuaternionConjugate(q_current));
        Eigen::Vector3d err_quat_vec(
            err_quat_.x(), err_quat_.y(), err_quat_.z());
        
        err_ang_vel_ = w_desired - w_current;
        integral_err_quat_ += Eigen::Vector3d(
            err_quat_.x(), err_quat_.y(), err_quat_.z()) * time_dt_;
        
        torque_command = angular_kp_ * err_quat_vec
                        + angular_kd_ * err_ang_vel_
                        + angular_ki_ * integral_err_quat_
                        + w_current.cross(inertia_ * w_current);
    }

    if (torque_command.norm() > max_torque_) {
        torque_command = (torque_command.normalized()) * max_torque_;
    }

    //----------------------End of Attitude Control-------------------//

    //---------------------- Linear Control--------------------------//
    Eigen::Vector3d pos_current(
        current_state.pose.position.x,
        current_state.pose.position.y,
        current_state.pose.position.z);
    
    Eigen::Vector3d speed_current(
        current_state.vel.linear.x,
        current_state.vel.linear.y,
        current_state.vel.linear.z);
    
    Eigen::Vector3d acc_command_inertial;
    Eigen::Vector3d acc_command_body;

    if (b_linear_guidance_active_)
    {
        acc_command_inertial = Eigen::Vector3d(
            current_guidance.accel.linear.x,
            current_guidance.accel.linear.y,
            current_guidance.accel.linear.z);
    } else
    {
        Eigen::Vector3d pos_desired(
            current_target.pose.position.x,
            current_target.pose.position.y,
            current_target.pose.position.z);
        
        Eigen::Vector3d speed_desired(
            current_target.vel.linear.x,
            current_target.vel.linear.y,
            current_target.vel.linear.z);

        err_pos_ = pos_desired - pos_current;
        err_vel_ = speed_desired - speed_current;
        integral_err_pos_ += err_pos_ * time_dt_;

        acc_command_inertial = linear_kp_ * err_pos_
                            + linear_kd_ * err_vel_
                            + linear_ki_ * integral_err_pos_;
    }

    Eigen::Matrix3d D_I2B = q_current.toRotationMatrix();
    acc_command_body = D_I2B * acc_command_inertial;
    Eigen::Vector3d force_command_body = mass_ * acc_command_body;

    if (force_command_body.norm() > max_force_) {
        force_command_body = (force_command_body.normalized()) * max_force_;
    }

    //----------------------End of Linear Control-------------------//

    // publish command
    o_command_.id = current_state.id;
    o_command_.force.x = force_command_body.x();
    o_command_.force.y = force_command_body.y();
    o_command_.force.z = force_command_body.z();
    o_command_.torque.x = torque_command.x();
    o_command_.torque.y = torque_command.y();
    o_command_.torque.z = torque_command.z();
    pub_command_->publish(o_command_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());

  rclcpp::shutdown();
  return 0;
}