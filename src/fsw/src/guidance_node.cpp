/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      guidance_node.cpp
 * @brief     6-DOF guidance node source file
 *
 * @date      2026-02-26 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#include "fsw/guidance_node.hpp"

Guidance::Guidance()
: Node("guidance_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Guidance node...");
    
    //QoS settings
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Declare Parameters
    this->declare_parameter("loop_rate_hz", loop_rate_hz_);

    this->declare_parameter("target_x", target_x_);
    this->declare_parameter("target_y", target_y_);
    this->declare_parameter("target_z", target_z_);
    this->declare_parameter("target_qx", target_qx_);
    this->declare_parameter("target_qy", target_qy_);
    this->declare_parameter("target_qz", target_qz_);
    this->declare_parameter("target_qw", target_qw_);

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
        "Target State: position=(%.3f, %.3f, %.3f),
        orientation(quaternion)=(%.3f, %.3f, %.3f, %.3f)",
        target_x_, target_y_, target_z_,
        target_qx_, target_qy_, target_qz_, target_qw_);

    // Subscribers Initialization
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state", qos_profile,
        std::bind(&Guidance::CallbackState, this, std::placeholders::_1));

    // Publishers Initialization
    pub_guidance_ = this->create_publisher<interfaces::msg::Command>(
        "command", qos_profile);
    
    // Timer Initialization
    rclcpp::Time current_time = steady_clock.now();

    // Run Contol Loop
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });
}

Guidance::~Guidance()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Guidance node...");
}

void Guidance::GetParameters()
{
    // fetch parameters and store them in member variables
    this->get_parameter("loop_rate_hz", loop_rate_hz_);

    this->get_parameter("target_x", target_x_);
    this->get_parameter("target_y", target_y_);
    this->get_parameter("target_z", target_z_);
    this->get_parameter("target_qx", target_qx_);
    this->get_parameter("target_qy", target_qy_);
    this->get_parameter("target_qz", target_qz_);
    this->get_parameter("target_qw", target_qw_);

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

void Guidance::Init(const interfaces::msg::State& initial_state)
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Guidance Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);

    // Initialize time
    sim_time_prev_ = initial_state.header.stamp;

    // Initialize guidance message
    o_guidance_.header.stamp = initial_state.header.stamp;
}

void Guidance::Run()
{
    // handle initialization
    if (!b_simulator_initialized_) {
        RCLCPP_WARN(this->get_logger(),
            "Waiting for simulator initialization...");
        return;
    }

    if (!b_guidance_initialized_) {
        Init(last_state_);
        b_guidance_initialized_ = true;
    }

    // get subscribed state
    interfaces::msg::State current_state;
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        current_state = last_state_;
        sim_time_curr_ = current_state.header.stamp;
    }

    // compute time step
    time_dt_ = (sim_time_curr_ - sim_time_prev_).seconds();
    sim_time_prev_ = sim_time_curr_;

    // attitude guidance
    err_quat_ = QuaternionSignCorrection(
        Eigen::Quaterniond(
            target_qw_, target_qx_, target_qy_, target_qz_) *
        QuaternionConjugate(Eigen::Quaterniond(
            current_state.pose.orientation.w,
            current_state.pose.orientation.x,
            current_state.pose.orientation.y,
            current_state.pose.orientation.z)));

    man_angle_ = 2.0 * std::acos(err_quat_.w());
    eigen_vec_ = Eigen::Vector3d(err_quat_.x(), err_quat_.y(), err_quat_.z());
    eigen_vec_.normalize();

    ang_vel_curr_ = Eigen::Vector3d(
        current_state.vel.angular.x,
        current_state.vel.angular.y,
        current_state.vel.angular.z);

    // angular acceleration limit (approximate)
    Eigen::Vector3d I_e_product_ = inertia_ * eigen_vec_;
    double ang_acc_limit = 0.8 * max_torque_ / I_e_product_.norm();

    
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());

  rclcpp::shutdown();
  return 0;
}