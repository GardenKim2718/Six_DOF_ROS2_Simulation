/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      actuator_node.cpp
 * @brief     6-DOF actuator node source file
 *
 * @date      2026-03-05 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-11 updated by Chungwon Kim to add control allocation for RWA and thrusters
 *            2026-03-23 updated by Chungwon Kim to update QoS policy
 */

#include "fsw/actuator_node.hpp"

Actuator::Actuator()
: Node("actuator_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Actuator node...");
    
    // Declare Parameters
    this->declare_parameter("loop_rate_hz", loop_rate_hz_);

    this->declare_parameter("max_thrust", max_thrust_);
    this->declare_parameter("max_rwa_momentum", max_rwa_momentum_);
    this->declare_parameter("max_rwa_torque", max_rwa_torque_);

    this->declare_parameter("lambda_qp", lambda_qp_);

    // Get parameters
    GetParameters();

    RCLCPP_INFO(this->get_logger(),
        "Control Node Parameters: loop_rate_hz=%.3f", loop_rate_hz_);

    RCLCPP_INFO(this->get_logger(),
        "Thruster Limit: max_thrust=%.3f", max_thrust_);

    RCLCPP_INFO(this->get_logger(),
        "RWA Parameters: max_momentum=%.3f, max_torque=%.3f",
        max_rwa_momentum_, max_rwa_torque_);

    RCLCPP_INFO(this->get_logger(),
        "Control Allocation Parameter: lambda_qp=%.3e", lambda_qp_);

    // QoS settings
    // Event Callbacks for QoS
    rclcpp::SubscriptionOptions sub_options;
    sub_options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineRequestedInfo & info)
        {
            RCLCPP_WARN(
            this->get_logger(),
            "Subscription deadline missed: total_count=%d, total_count_change=%d",
            info.total_count,
            info.total_count_change);
        };

    // Publisher QoS
    auto qos_profile_pub = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile_pub.reliable();
    qos_profile_pub.transient_local();
    qos_profile_pub.deadline(rclcpp::Duration::from_seconds(1.0 / loop_rate_hz_));

    // Subscriber QoS
    auto qos_profile_sub = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile_sub.reliable();
    qos_profile_sub.transient_local();
    qos_profile_sub.deadline(rclcpp::Duration::from_seconds(1.0 / loop_rate_hz_ * 1.2));

    // Subscribers Initialization
    sub_command_ = this->create_subscription<interfaces::msg::Command>(
        "command", qos_profile_sub,
        std::bind(&Actuator::CallbackCommand, this, std::placeholders::_1),
        sub_options);

    sub_navigation_ = this->create_subscription<interfaces::msg::Navigation>(
        "navigation", qos_profile_sub,
        std::bind(&Actuator::CallbackNavigation, this, std::placeholders::_1),
        sub_options);

    // Publishers Initialization
    pub_actuator_ = this->create_publisher<interfaces::msg::Actuator>(
        "actuator", qos_profile_pub);

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

    this->get_parameter("max_thrust", max_thrust_);
    this->get_parameter("max_rwa_momentum", max_rwa_momentum_);
    this->get_parameter("max_rwa_torque", max_rwa_torque_);
    this->get_parameter("lambda_qp", lambda_qp_);
}

void Actuator::Init()
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Actuator Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);

    // Center of Mass Configuration
    center_of_mass_ << 0.0, 0.0, 0.0;

    // RWA configuration
    rwa_mounting_matrix_ <<
        -1, -1,  1,  1,
        -1,  1,  1, -1,
         1,  1,  1,  1;
    
    for (int i = 0; i < rwa_mounting_matrix_.cols(); ++i)
    {
        const double n = rwa_mounting_matrix_.col(i).norm();
        if (n > 1e-12)
        {
            rwa_mounting_matrix_.col(i) /= n;
        }
    }

    rwa_mounting_matrix_pseudo_inverse_ = 
        rwa_mounting_matrix_.transpose() *
        (rwa_mounting_matrix_ * rwa_mounting_matrix_.transpose()).inverse();

    // thruster configuration
    thruster_positions_ <<
        -0.15, -0.15,  -0.15,  0.15, -0.15,  0.15,  0.15, -0.15, -0.15,  0.15,  0.15, -0.15,
        0.15, -0.15,  0.15, -0.15, -0.15, -0.15,  0.15,  0.15,  0.15, -0.15,  0.15, -0.15,
        0.15, -0.15, -0.15,  0.15,  0.15, -0.15,  0.15, -0.15,  0.15,  0.15, -0.15, -0.15;

    thruster_directions_ <<
        1,  1, -1, -1,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  1,  1, -1, -1,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0, -1, -1,  1,  1;

    // thruster direction normalization
    for (int i = 0; i < thruster_directions_.cols(); ++i)
    {
        const double n = thruster_directions_.col(i).norm();
        if (n > 1e-12)
        {
            thruster_directions_.col(i) /= n;
        }
    }

    // QP solver initialization
    thruster_qp_solver_.reset(12, 0, 24);
}

void Actuator::Run()
{
    if (!b_navigation_initialized_ || !b_control_initialized_) {
        RCLCPP_WARN(this->get_logger(),
            "Waiting for navigation and control initialization...");
        return;
    }

    // handle initialization
    if (!b_actuator_initialized_) {
        Init();
        b_actuator_initialized_ = true;
    }

    // get subscribed command
    interfaces::msg::Command current_command;
    {
        std::lock_guard<std::mutex> lock(mutex_command_);
        current_command = last_command_;
    }

    // get subscribed navigation state
    interfaces::msg::Navigation current_state;
    {
        std::lock_guard<std::mutex> lock(mutex_navigation_);
        current_state = last_state_;
    }

    Eigen::Vector4d rwa_momentum;
    rwa_momentum << current_state.rwa_momentum.data()[0],
                    current_state.rwa_momentum.data()[1],
                    current_state.rwa_momentum.data()[2],
                    current_state.rwa_momentum.data()[3];

    // compute actuator command from control command
    //------------------------RWA Control Allocation------------------------//
    // use RWA prior to thruster for torque generation
    Eigen::Vector3d torque_cmd(
        current_command.torque.x,
        current_command.torque.y,
        current_command.torque.z
    );

    Eigen::Vector4d rwa_torque_cmd = rwa_mounting_matrix_pseudo_inverse_ * torque_cmd;
    double max_rwa_t = std::max(std::abs(rwa_torque_cmd.maxCoeff()),
                                std::abs(rwa_torque_cmd.minCoeff()));

    if(max_rwa_t > max_rwa_torque_)  // scale down for RWA torque limit
    {
        rwa_torque_cmd = (max_rwa_torque_/max_rwa_t) * rwa_torque_cmd;
    }

    for (int i = 0; i < 4; ++i) {   // check for RWA momentum saturation
        if ((std::abs(rwa_momentum(i)) >= max_rwa_momentum_) &&
            (rwa_momentum(i)*rwa_torque_cmd(i) < 0.0))
        {
            if (abs(rwa_momentum(i)) > max_rwa_momentum_) {
                rwa_torque_cmd(i) = 0.0;
            }
        }
    }
    Eigen::Vector3d rwa_torque_real = rwa_mounting_matrix_ * rwa_torque_cmd;
    //-----------------end of RWA Control Allocation------------------------//

    //---------------------Thruster Control Allocation----------------------//
    // construct thruster B matrix
    for (int i = 0; i < 12; ++i) {
        Eigen::Vector3d r_i = thruster_positions_.col(i) - center_of_mass_;
        Eigen::Vector3d d_i = thruster_directions_.col(i);
        B_thruster_.block<3,1>(0,i) = d_i;
        B_thruster_.block<3,1>(3,i) = r_i.cross(d_i);
    }

    // control allocation using QP solver
    Eigen::Vector3d force_cmd(
        current_command.force.x,
        current_command.force.y,
        current_command.force.z
    );

    Eigen::Vector3d thruster_torque_cmd = torque_cmd - rwa_torque_real;

    Eigen::Matrix<double, 6, 1> wrench_cmd;
    wrench_cmd << force_cmd, thruster_torque_cmd;

    Eigen::Matrix<double, 12,1> thruster_cmd;
    bool qp_success = SolveThrusterAllocationQP(wrench_cmd, thruster_cmd);

    if (!qp_success) {
        RCLCPP_ERROR(this->get_logger(),
            "Thruster allocation QP failed. Commanding zero thrust.");
        thruster_cmd.setZero();
    }
    //-----------------end of Thruster Control Allocation----------------------//

    // publish actuator command
    o_actuator_.id = current_command.id;

    for (int i = 0; i < 12; ++i) {
        o_actuator_.thruster_cmd[i] = thruster_cmd(i);
    }

    for (int i = 0; i < 4; ++i) {
        o_actuator_.rwa_torque_cmd[i] = rwa_torque_cmd(i);
    }

    pub_actuator_->publish(o_actuator_);
}

bool Actuator::SolveThrusterAllocationQP(
    const Eigen::Matrix<double, 6, 1>& wrench_cmd,
    Eigen::Matrix<double, 12, 1>& thruster_cmd)
{
    constexpr int n_var = 12;
    constexpr int n_ineq = 24;

    using Matrix12d = Eigen::Matrix<double, 12, 12>;
    using Vector12d = Eigen::Matrix<double, 12, 1>;
    using MatrixCI  = Eigen::Matrix<double, 24, 12>;
    using VectorCI  = Eigen::Matrix<double, 24, 1>;

    Matrix12d H = B_thruster_.transpose() * B_thruster_
                + lambda_qp_ * Matrix12d::Identity();

    Vector12d g0 = -B_thruster_.transpose() * wrench_cmd;

    Eigen::MatrixXd CE(0, n_var);
    Eigen::VectorXd ce0(0);

    MatrixCI CI;
    VectorCI ci0;
    CI.setZero();
    ci0.setZero();

    // lower bounds: u_i >= 0
    for (int i = 0; i < n_var; ++i) {
        CI(i, i) = 1.0;
        ci0(i) = 0.0;
    }

    // upper bounds: -u_i + max_thrust_ >= 0
    for (int i = 0; i < n_var; ++i) {
        CI(n_var + i, i) = -1.0;
        ci0(n_var + i) = max_thrust_;
    }

    Eigen::VectorXd x_qp(n_var);
    x_qp.setZero();

    auto status = thruster_qp_solver_.solve_quadprog(
        H, g0, CE, ce0, CI, ci0, x_qp);

    if (status != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
        RCLCPP_ERROR(this->get_logger(),
            "eiquadprog failed, status=%d", static_cast<int>(status));
        thruster_cmd.setZero();
        return false;
    }

    thruster_cmd = x_qp;

    for (int i = 0; i < n_var; ++i) {
        if (thruster_cmd(i) < 1e-10) {
            thruster_cmd(i) = 0.0;
        }
        if (thruster_cmd(i) > max_thrust_) {
            thruster_cmd(i) = max_thrust_;
        }
    }

    return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Actuator>());

  rclcpp::shutdown();
  return 0;
}