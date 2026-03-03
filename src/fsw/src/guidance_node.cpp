/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      guidance_node.cpp
 * @brief     6-DOF guidance node source file
 *
 * @date      2026-02-26 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-01 edited by Chungwon Kim (added linear guidance logic based on Apollo Powered Descent Guidance)
 *            2026-03-03 edited by Chungwon Kim (added rotational guidance logic based on Apollo Powered Descent Guidance application on attitude guidance)
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

    this->declare_parameter("angular_kp", angular_kp_);
    this->declare_parameter("angular_kd", angular_kd_);

    // Get parameters
    GetParameters();

    RCLCPP_INFO(this->get_logger(),
        "Control Node Parameters: loop_rate_hz=%.3f", loop_rate_hz_);
    
    RCLCPP_INFO(
        this->get_logger(),
        "Target State: position=(%.3f, %.3f, %.3f), "
        "linear speed=(%.3f, %.3f, %.3f), "
        "orientation(quaternion)=(%.3f, %.3f, %.3f, %.3f), "
        "angular speed=(%.3f, %.3f, %.3f)",
        target_x_, target_y_, target_z_,
        target_vx_, target_vy_, target_vz_,
        target_qx_, target_qy_, target_qz_, target_qw_,
        target_wx_, target_wy_, target_wz_);

    // Subscribers Initialization
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state", qos_profile,
        std::bind(&Guidance::CallbackState, this, std::placeholders::_1));

    // Publishers Initialization
    pub_guidance_ = this->create_publisher<interfaces::msg::Guidance>(
        "guidance", qos_profile);
    
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

    this->get_parameter("angular_kp", angular_kp_);
    this->get_parameter("angular_kd", angular_kd_);

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
    o_target_.header.frame_id = initial_state.header.frame_id;
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

    // linear acceleration limit (approximate, with 15% margin)
    double acc_limit_ = 0.85 * max_force_ / mass_;

    T_go_linear_ = T_go_linear_ - time_dt_;   // decrement time-to-go guess by time step

    if (b_linear_guidance_active_){
        LinearGuidance(current_state, o_target_, T_go_linear_, acc_limit_,
            accel_cmd_, b_linear_guidance_active_);
    } else {
        accel_cmd_ = Eigen::Vector3d::Zero();
    }

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

    double maneuver_angle_ = 2.0 * std::acos(std::abs(err_quat_.w()));

    // compute angular acceleration limit (approximate, 40% margin)
    Eigen::Vector3d I_e_ = inertia_ * eigen_vec_;
    double ang_acc_limit = 0.6 * max_torque_ / I_e_.norm();

    if (b_angular_guidance_active_){
        AngularGuidance(current_state, o_target_, T_go_angular_, ang_acc_limit,
            ang_accel_cmd_, b_angular_guidance_active_);
    } else {
        ang_accel_cmd_ = Eigen::Vector3d::Zero();
    }
    //--------------end of rotational guidance logic------------------------//

    // publish guidance command
    o_guidance_.header.stamp = current_state.header.stamp;
    o_guidance_.linear_guidance_active = b_linear_guidance_active_;
    o_guidance_.angular_guidance_active = b_angular_guidance_active_;
    o_guidance_.accel.linear.x = accel_cmd_.x();
    o_guidance_.accel.linear.y = accel_cmd_.y();
    o_guidance_.accel.linear.z = accel_cmd_.z();
    o_guidance_.accel.angular.x = ang_accel_cmd_.x();
    o_guidance_.accel.angular.y = ang_accel_cmd_.y();
    o_guidance_.accel.angular.z = ang_accel_cmd_.z();
    
    pub_guidance_->publish(o_guidance_);

    // publish target state
    pub_target_->publish(o_target_);
}

void Guidance::LinearGuidance(
    const interfaces::msg::State& current_state,
    const interfaces::msg::Target& target_state,
    double& T_go_linear_, const double acc_limit_,
    Eigen::Vector3d& accel_cmd_,
    bool &b_linear_guidance_active_)
{   
    // based the Apollo Powered Descent Guidance (APDG)
    // iterate until the trajectory does not exceed the acceleration limit

    Eigen::Vector3d x0 = Eigen::Vector3d(
        current_state.pose.position.x,
        current_state.pose.position.y,
        current_state.pose.position.z);
    Eigen::Vector3d v0 = Eigen::Vector3d(
        current_state.vel.linear.x,
        current_state.vel.linear.y,
        current_state.vel.linear.z);
    Eigen::Vector3d xf = Eigen::Vector3d(
        target_state.pose.position.x,
        target_state.pose.position.y,
        target_state.pose.position.z);
    Eigen::Vector3d vf = Eigen::Vector3d(
        target_state.vel.linear.x,
        target_state.vel.linear.y,
        target_state.vel.linear.z);
    Eigen::Vector3d err_vel = Eigen::Vector3d(
        target_state.vel.linear.x - current_state.vel.linear.x,
        target_state.vel.linear.y - current_state.vel.linear.y,
        target_state.vel.linear.z - current_state.vel.linear.z);

    // running bisection method to find the time-to-go that satisfies the acceleration limit
    double Tgo_guess_ = T_go_linear_;
    double Tgo_guess2_ = T_go_linear_;
    
    const double Tgo_delta_ = 2.0;   // time-to-go adjustment step [s]
    const double Tgo_tol = 0.1;       // time-to-go convergence tolerance [s]

    bool Tgo_converged = false;
    bool Guess1_ = false;
    bool Guess2_ = false;
    
    Guess1_ = ApolloPoweredDescentGuidanceValidate(x0, xf, v0, vf, Tgo_guess_, acc_limit_);
    Guess2_ = Guess1_;

    while (Guess2_ == Guess1_)
    {
        if (Guess1_)
        {
            Tgo_guess2_ = Tgo_guess2_ - Tgo_delta_;

            if (Tgo_guess2_ < T_go_linear_min_){
                Tgo_converged = true;
                b_linear_guidance_active_ = false;

                T_go_linear_ = T_go_linear_min_;
                break;
            }
        } else
        {
            Tgo_guess2_ = Tgo_guess2_ + Tgo_delta_;
        }
        Guess2_ = ApolloPoweredDescentGuidanceValidate(x0, xf, v0, vf, Tgo_guess2_, acc_limit_);
    }

    while (!Tgo_converged)
    {
        double Tgo_guess_mid_ = 0.5 * (Tgo_guess_ + Tgo_guess2_);
        Guess1_ = ApolloPoweredDescentGuidanceValidate(x0, xf, v0, vf, Tgo_guess_mid_, acc_limit_);

        if (abs(Tgo_guess_ - Tgo_guess2_) < Tgo_tol){
            Tgo_converged = true;
            T_go_linear_ = Tgo_guess_mid_;
        }

        if (Guess1_){
            Tgo_guess2_ = Tgo_guess_mid_;
        } else {
            Tgo_guess_ = Tgo_guess_mid_;
        }
    }

    // compute reference acceleration command with the converged time-to-go
    // (inertial frame)
    accel_cmd_ = 6.0 * (xf - x0 - v0 * T_go_linear_) / (T_go_linear_ * T_go_linear_) - 
                 2.0 * (vf - v0) / T_go_linear_;
}

bool Guidance::ApolloPoweredDescentGuidanceValidate(
        const Eigen::Vector3d &x0, const Eigen::Vector3d &xf,
        const Eigen::Vector3d &v0, const Eigen::Vector3d &vf,
        const double T_go_, const double acc_limit_)
{
    // checking whether the given Tgo satisfies the acceleration limit for the given state error
    Eigen::Vector3d c0 = 6.0 * (xf - x0 - v0 * T_go_) / (T_go_ * T_go_) - 
                        2.0 * (vf - v0) / T_go_;
    Eigen::Vector3d c1 = 6.0 * (vf - v0) / (T_go_ * T_go_) - 12.0 * (xf - x0 - v0 * T_go_) / (T_go_ * T_go_ * T_go_);

    // check for maximum acceleration magnitude
    double max_acc_cand1 = c0.norm();
    double max_acc_cand2 = (c0 + c1 * T_go_).norm();
    double max_acc = std::max(max_acc_cand1, max_acc_cand2);

    return max_acc < acc_limit_;
}

void Guidance::AngularGuidance(
    const interfaces::msg::State &current_state,
    const interfaces::msg::Target &target_state,
    double &T_go_angular_, const double ang_acc_limit_,
    Eigen::Vector3d &ang_accel_cmd_,
    bool &b_angular_guidance_active_)
{
    // based on Apollo Powered Descent Guidance application on attitude guidance
    // see for below paper for details:
    // "GUIDANCE, NAVIGATION, AND CONTROL OF SMALL SATELLITE ATTITUDE USING MICRO-THRUSTERS"
    // modified for Quaternion attitude representation by Chungwon Kim

    // create target angular acceleration with PD control
    Eigen::Vector3d err_quat_vec_ = Eigen::Vector3d(err_quat_.x(), err_quat_.y(), err_quat_.z());
    Eigen::Vector3d ang_accel_tgt_ = angular_kp_ * err_quat_vec_ + angular_kd_ * err_ang_vel_;

    // set T_go for angular guidance with T_go from linear guidance (temporary)
    T_go_angular_ = T_go_linear_ * 0.8;
    
    if (T_go_angular_ < T_go_angular_min_){
        T_go_angular_ = T_go_angular_min_;
        b_angular_guidance_active_ = false;
    }

    // compute angular acceleration command (body frame)
    ang_accel_cmd_ = 12.0 * err_quat_vec_ / (T_go_angular_ * T_go_angular_) + 
                     6.0 * (target_ang_vel_ - curr_ang_speed_) / T_go_angular_ + 
                     ang_accel_tgt_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Guidance>());

  rclcpp::shutdown();
  return 0;
}