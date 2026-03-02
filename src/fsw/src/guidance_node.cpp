/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      guidance_node.cpp
 * @brief     6-DOF guidance node source file
 *
 * @date      2026-02-26 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-01 expanded by Chungwon Kim for 6-DOF guidance
 *                       (applied augmented Apollo powered descent guidance)
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
    this->declare_parameter("target_vx", target_vx_);
    this->declare_parameter("target_vy", target_vy_);
    this->declare_parameter("target_vz", target_vz_);
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
        linear speed=(%.3f, %.3f, %.3f),
        orientation(quaternion)=(%.3f, %.3f, %.3f, %.3f),
        angular speed=(%.3f, %.3f, %.3f)",
        target_x_, target_y_, target_z_,
        target_vx_, target_vy_, target_vz_,
        target_qx_, target_qy_, target_qz_, target_qw_,
        target_wx_, target_wy_, target_wz_);

    // Subscribers Initialization
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state", qos_profile,
        std::bind(&Guidance::CallbackState, this, std::placeholders::_1));

    // Publishers Initialization
    pub_guidance_ = this->create_publisher<interfaces::msg::Command>(
        "command", qos_profile);
    
    pub_target_ = this->create_publisher<interfaces::msg::Target>(
        "target", qos_profile);
    
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
    this->get_parameter("target_vx", target_vx_);
    this->get_parameter("target_vy", target_vy_);
    this->get_parameter("target_vz", target_vz_);

    this->get_parameter("target_qx", target_qx_);
    this->get_parameter("target_qy", target_qy_);
    this->get_parameter("target_qz", target_qz_);
    this->get_parameter("target_qw", target_qw_);
    this->get_parameter("target_wx", target_wx_);
    this->get_parameter("target_wy", target_wy_);
    this->get_parameter("target_wz", target_wz_);

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

    // target state
    target_pos_ = Eigen::Vector3d(target_x_, target_y_, target_z_);
    target_speed_ = Eigen::Vector3d(target_vx_, target_vy_, target_vz_);
    target_quat_ = Eigen::Quaterniond(target_qw_, target_qx_, target_qy_, target_qz_);
    target_ang_vel_ = Eigen::Vector3d(target_wx_, target_wy_, target_wz_);

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
    o_guidance_.id = initial_state.id;
    o_guidance_.header.stamp = initial_state.header.stamp;

    // Initialize target message
    o_target_.id = initial_state.id;
    o_target_.header.stamp = initial_state.header.stamp;
    o_target_.pose.position.x = target_x_;
    o_target_.pose.position.y = target_y_;
    o_target_.pose.position.z = target_z_;
    o_target_.pose.orientation.x = target_qx_;
    o_target_.pose.orientation.y = target_qy_;
    o_target_.pose.orientation.z = target_qz_;
    o_target_.pose.orientation.w = target_qw_;
    o_target_.vel.linear.x = target_vx_;
    o_target_.vel.linear.y = target_vy_;
    o_target_.vel.linear.z = target_vz_;
    o_target_.vel.angular.x = target_wx_;
    o_target_.vel.angular.y = target_wy_;
    o_target_.vel.angular.z = target_wz_;
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


    //----------------------Linear Guidance Logic ---------------------------//
    // Based on Apollo Powered Descent Guidance
    // modified for zero-G environment

    // linear acceleration limit (approximate)
    double acc_limit = 0.85 * max_force_ / mass_;

    
    //-------------------end of linear guidance logic------------------------//


    //--------------------Rotational Guidance Logic -------------------------//
    // attitude error
    curr_quat_ = Eigen::Quaterniond(
        current_state.pose.orientation.w,
        current_state.pose.orientation.x,
        current_state.pose.orientation.y,
        current_state.pose.orientation.z);
    curr_quat_.normalize();

    curr_ang_speed_ = Eigen::Vector3d(
        current_state.vel.angular.x,
        current_state.vel.angular.y,
        current_state.vel.angular.z);

    err_quat_ = QuaternionSignCorrection(target_quat_ * QuaternionConjugate(curr_quat_));
    err_ang_vel_ = target_ang_vel_ - curr_ang_speed_;
    
    eigen_vec_ = Eigen::Vector3d(err_quat_.x(), err_quat_.y(), err_quat_.z());
    eigen_vec_.normalize();

    maneuver_angle_ = 2.0 * std::acos(std::abs(err_quat_.w()));

    // compute angular acceleration limit (approximate)
    I_e_ = inertia_ * eigen_vec_;
    double ang_acc_limit = 0.85 * max_torque_ / I_e_.norm();

    //--------------end of rotational guidance logic------------------------//

    // publish guidance command
    pub_guidance_->publish(o_guidance_);

    // publish target state
    pub_target_->publish(o_target_);
}

void Guidance::LinearGuidance(
    const interfaces::msg::State& current_state,
    const interfaces::msg::Target& target_state)
{
    // based the Apollo Powered Descent Guidance (APDG)
    // iterate until the trajectory does not exceed the acceleration limit

    // 1) Extract current state (position & velocity)
    Eigen::Vector3d r(
        current_state.pose.position.x,
        current_state.pose.position.y,
        current_state.pose.position.z);

    Eigen::Vector3d v(
        current_state.vel.linear.x,
        current_state.vel.linear.y,
        current_state.vel.linear.z);

    // 2) Extract target state (position & velocity)
    Eigen::Vector3d r_f(
        target_state.pose.position.x,
        target_state.pose.position.y,
        target_state.pose.position.z);

    Eigen::Vector3d v_f(
        target_state.vel.linear.x,
        target_state.vel.linear.y,
        target_state.vel.linear.z);

    // 3) Acceleration limit (already defined logic)
    const double acc_limit = 0.85 * max_force_ / mass_;
    if (acc_limit <= 1e-9) {
        RCLCPP_WARN(this->get_logger(),
            "Acceleration limit is near zero (mass=%.3f, max_force=%.3f).",
            mass_, max_force_);
        // Just zero command if config is degenerate
        // TODO: set command to zero in o_guidance_
        return;
    }

    // 4) Initial guess for T_go
    Eigen::Vector3d dr = r_f - r;
    Eigen::Vector3d dv = v_f - v;
    const double dr_norm = dr.norm();
    const double dv_norm = dv.norm();

    double T_go = tgo_last_;  // start from last used value

    // A heuristic update from errors + limits for robustness
    if (acc_limit > 1e-9) {
        double T_pos = (dr_norm > 1e-6)
            ? std::sqrt(4.0 * dr_norm / acc_limit)
            : tgo_min_;
        double T_vel = (dv_norm > 1e-6)
            ? 2.0 * dv_norm / acc_limit
            : tgo_min_;

        double T_guess = std::max(T_pos, T_vel);
        T_go = std::max(tgo_min_, std::min(T_guess, tgo_max_));
    }

    // 5) Iterate T_go until acceleration satisfies limit
    Eigen::Vector3d a_cmd = Eigen::Vector3d::Zero();
    for (int k = 0; k < tgo_max_iter_; ++k) {
        a_cmd = ApolloPoweredDescentGuidanceAccel(r, v, r_f, v_f, T_go);
        double a_norm = a_cmd.norm();

        if (a_norm <= acc_limit || T_go >= tgo_max_) {
            break;
        }

        // Increase T_go to reduce required acceleration
        // (multiplicative factor is simple & robust)
        T_go *= 1.2;
        if (T_go > tgo_max_) {
            T_go = tgo_max_;
        }
    }

    // Store T_go for next cycle continuity
    tgo_last_ = T_go;

    // 6) Final safety saturation if necessary
    double a_norm = a_cmd.norm();
    if (a_norm > acc_limit && a_norm > 1e-9) {
        a_cmd *= (acc_limit / a_norm);
    }

    // 7) Convert acceleration to force (if Command is force-based)
    Eigen::Vector3d f_cmd = mass_ * a_cmd;

}

Eigen::Vector3d Guidance::ApolloPoweredDescentGuidanceAccel(
    const Eigen::Vector3d& r,
    const Eigen::Vector3d& v,
    const Eigen::Vector3d& r_f,
    const Eigen::Vector3d& v_f,
    double T_go) const
{
    // Protect against degenerate T_go
    if (T_go < 1e-6) {
        return Eigen::Vector3d::Zero();
    }

    // Zero-g ZEM / ZEV
    Eigen::Vector3d ZEM = r_f - (r + v * T_go);
    Eigen::Vector3d ZEV = v_f - v;

    // APDG-style acceleration (no gravity term)
    Eigen::Vector3d a_cmd =
        - apdg_kR_ * (ZEM / (T_go * T_go))
        - apdg_kV_ * (ZEV / T_go);

    return a_cmd;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Guidance>());

  rclcpp::shutdown();
  return 0;
}