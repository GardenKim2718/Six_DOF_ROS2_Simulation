/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      evaluation_node.cpp
 * @brief     6-DOF evaluation node source file
 *
 * @date      2026-03-12 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#include "evaluation/evaluation_node.hpp"

using interfaces::msg::State;
using interfaces::msg::Actuator;
using interfaces::msg::Command;
using interfaces::msg::Guidance;
using interfaces::msg::Target;

Evaluation::Evaluation()
: Node("evaluation_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Evaluation node...");
    
    // Declare Parameters
    this->declare_parameter<double>("loop_rate_hz", loop_rate_hz_);

    this->declare_parameter<double>("mass", 1.0);
    this->declare_parameter<double>("inertia_xx", 1.0);
    this->declare_parameter<double>("inertia_yy", 1.0);
    this->declare_parameter<double>("inertia_zz", 1.0);
    this->declare_parameter<double>("inertia_xy", 0.0);
    this->declare_parameter<double>("inertia_xz", 0.0);
    this->declare_parameter<double>("inertia_yz", 0.0);
    this->declare_parameter<std::vector<double>>("center_of_mass",
        std::vector<double>{0.0, 0.0, 0.0});

    // Get parameters
    get_parameters();

    RCLCPP_INFO(this->get_logger(),
        "Evaluation Node Parameters: loop_rate_hz=%.3f", loop_rate_hz_);

    RCLCPP_INFO(this->get_logger(),
                "Mass & Inertia: mass=%.3f, Ixx=%.3f, Iyy=%.3f, Izz=%.3f, Ixy=%.3f, Ixz=%.3f, Iyz=%.3f",
                mass_, inertia_(0, 0), inertia_(1, 1), inertia_(2, 2),
                inertia_(0, 1), inertia_(0, 2), inertia_(1, 2));

    RCLCPP_INFO(this->get_logger(),
        "Center of Mass: x=%.3f, y=%.3f, z=%.3f",
        center_of_mass_(0), center_of_mass_(1), center_of_mass_(2));

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
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state", qos_profile_sub,
        std::bind(&Evaluation::callback_state, this, std::placeholders::_1),
        sub_options);

    sub_actuator_ = this->create_subscription<interfaces::msg::Actuator>(
        "actuator", qos_profile_sub,
        std::bind(&Evaluation::callback_actuator, this, std::placeholders::_1),
        sub_options);

    sub_guidance_ = this->create_subscription<interfaces::msg::Guidance>(
        "guidance", qos_profile_sub,
        std::bind(&Evaluation::callback_guidance, this, std::placeholders::_1),
        sub_options);

    sub_command_ = this->create_subscription<interfaces::msg::Command>(
        "command", qos_profile_sub,
        std::bind(&Evaluation::callback_command, this, std::placeholders::_1),
        sub_options);

    sub_target_ = this->create_subscription<interfaces::msg::Target>(
        "target", qos_profile_sub,
        std::bind(&Evaluation::callback_target, this, std::placeholders::_1),
        sub_options);

    // Publishers Initialization
    pub_evaluation_ = this->create_publisher<interfaces::msg::Evaluation>(
        "evaluation", qos_profile_pub);

    // Steady clock initialization
    steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Timer Initialization
    rclcpp::Time current_time = steady_clock_->now();

    // run Evaluation Loop
    const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / loop_rate_hz_));

    t_run_node_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        steady_clock_,
        period_ns,
        std::bind(&Evaluation::run, this)
    );
}

Evaluation::~Evaluation()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Evaluation node...");
}

void Evaluation::get_parameters()
{
    // fetch parameters and store them in member variables
    this->get_parameter("loop_rate_hz", loop_rate_hz_);

    this->get_parameter("mass", mass_);
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    this->get_parameter("inertia_xx", Ixx);
    this->get_parameter("inertia_yy", Iyy);
    this->get_parameter("inertia_zz", Izz);
    this->get_parameter("inertia_xy", Ixy);
    this->get_parameter("inertia_xz", Ixz);
    this->get_parameter("inertia_yz", Iyz);

    std::vector<double> center_of_mass(3);
    this->get_parameter("center_of_mass", center_of_mass);

    center_of_mass_ = Eigen::Vector3d(center_of_mass[0], center_of_mass[1], center_of_mass[2]);

    // fill in the MOI matrix
    inertia_ << Ixx, Ixy, Ixz,
                Ixy, Iyy, Iyz,
                Ixz, Iyz, Izz;

    // compute inverse MOI matrix
    inertia_inv_ = inertia_.inverse();
}

void Evaluation::init()
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Evaluation Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);

    // Thruster configuration
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
}

void Evaluation::run()
{
    // handle initialization
    if (!b_evaluation_initialized_) {
        init();
        b_evaluation_initialized_ = true;
    }

    // get subscribed state
    State current_state;
    if (b_simulator_initialized_ == true){
        {
            std::lock_guard<std::mutex> lock(mutex_state_);
            current_state = last_state_;
        }
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *steady_clock_, 1000,
            "Waiting for Simulator initialization...");
    }

    Actuator current_actuator;
    if (b_actuator_initialized_ == true){
        {
            std::lock_guard<std::mutex> lock(mutex_actuator_);
            current_actuator = last_actuator_;
        }
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *steady_clock_, 1000,
            "Waiting for Actuator node initialization...");
    }

    Command current_command;
    if (b_control_initialized_ == true){
        {
            std::lock_guard<std::mutex> lock(mutex_command_);
            current_command = last_command_;
        }
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *steady_clock_, 1000,
            "Waiting for Command node initialization...");
    }

    Guidance current_guidance;
    if (b_guidance_initialized_ == true){
        {
            std::lock_guard<std::mutex> lock(mutex_guidance_);
            current_guidance = last_guidance_;
        }
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *steady_clock_, 1000,
            "Waiting for Guidance node initialization...");
    }

    Target current_target;
    if (b_target_initialized_ == true){
        {
            std::lock_guard<std::mutex> lock(mutex_target_);
            current_target = last_target_;
        }
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *steady_clock_, 1000,
            "Waiting for Guidance node for publishing target state...");
    }

    // Analyzing Guidance, Control and Control Allocation
    // Computing generated forces and torques from actuator outputs
    Eigen::Vector4d rwa_momentum;
    rwa_momentum << current_state.rwa_momentum[0],
                    current_state.rwa_momentum[1],
                    current_state.rwa_momentum[2],
                    current_state.rwa_momentum[3];
    
    Eigen::Vector4d rwa_torque_cmd;
    rwa_torque_cmd << current_actuator.rwa_torque_cmd[0],
                      current_actuator.rwa_torque_cmd[1],
                      current_actuator.rwa_torque_cmd[2],
                      current_actuator.rwa_torque_cmd[3];
    
    Eigen::VectorXd thruster_cmd(12);
    thruster_cmd << current_actuator.thruster_cmd[0],
                    current_actuator.thruster_cmd[1],
                    current_actuator.thruster_cmd[2],
                    current_actuator.thruster_cmd[3],
                    current_actuator.thruster_cmd[4],
                    current_actuator.thruster_cmd[5],
                    current_actuator.thruster_cmd[6],
                    current_actuator.thruster_cmd[7],
                    current_actuator.thruster_cmd[8],
                    current_actuator.thruster_cmd[9],
                    current_actuator.thruster_cmd[10],
                    current_actuator.thruster_cmd[11];
    
    Eigen::Matrix<double, 3, 12> B_thruster_torque;
    for (int i = 0; i < 12; ++i) {
        B_thruster_torque.col(i) = 
        (thruster_positions_.col(i) - center_of_mass_).cross(thruster_directions_.col(i));
    }

    Eigen::Vector3d rwa_torque = rwa_mounting_matrix_ * rwa_torque_cmd;
    Eigen::Vector3d thruster_force = thruster_directions_ * thruster_cmd;
    Eigen::Vector3d thruster_torque = B_thruster_torque * thruster_cmd;

    Eigen::Vector3d torque_actual = rwa_torque + thruster_torque;

    // compute guidance acceleration command vs actual acceleration
    Eigen::Quaterniond q(current_state.pose.orientation.w,
                         current_state.pose.orientation.x,
                         current_state.pose.orientation.y,
                         current_state.pose.orientation.z);
    q.normalize();

    const Eigen::Matrix3d D_B2I = q.toRotationMatrix().transpose();

    Eigen::Vector3d accel_actual = (D_B2I * thruster_force) / mass_;

    Eigen::Vector3d w_B;
    w_B << current_state.vel.angular.x,
           current_state.vel.angular.y,
           current_state.vel.angular.z;

    Eigen::Vector3d h_body = inertia_ * w_B + rwa_mounting_matrix_ * rwa_momentum;
    Eigen::Vector3d ang_accel_actual = inertia_inv_ * (torque_actual - w_B.cross(h_body));

    // Publish evaluation message
    o_evaluation_.id = current_state.id;
    o_evaluation_.header = current_state.header;

    o_evaluation_.force_command = current_command.force;
    o_evaluation_.torque_command = current_command.torque;

    o_evaluation_.force_actual.x = thruster_force.x();
    o_evaluation_.force_actual.y = thruster_force.y();
    o_evaluation_.force_actual.z = thruster_force.z();

    o_evaluation_.torque_actual.x = torque_actual.x();
    o_evaluation_.torque_actual.y = torque_actual.y();
    o_evaluation_.torque_actual.z = torque_actual.z();

    o_evaluation_.rwa_torque.x = rwa_torque.x();
    o_evaluation_.rwa_torque.y = rwa_torque.y();
    o_evaluation_.rwa_torque.z = rwa_torque.z();
    
    o_evaluation_.thruster_torque.x = thruster_torque.x();
    o_evaluation_.thruster_torque.y = thruster_torque.y();
    o_evaluation_.thruster_torque.z = thruster_torque.z();

    o_evaluation_.guidance_accel = current_guidance.accel;
    o_evaluation_.actual_accel.linear.x = accel_actual.x();
    o_evaluation_.actual_accel.linear.y = accel_actual.y();
    o_evaluation_.actual_accel.linear.z = accel_actual.z();
    o_evaluation_.actual_accel.angular.x = ang_accel_actual.x();
    o_evaluation_.actual_accel.angular.y = ang_accel_actual.y();
    o_evaluation_.actual_accel.angular.z = ang_accel_actual.z();

    pub_evaluation_->publish(o_evaluation_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Evaluation>());

  rclcpp::shutdown();
  return 0;
}