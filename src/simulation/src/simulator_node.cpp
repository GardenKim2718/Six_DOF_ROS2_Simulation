/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      simulator_node.cpp
 * @brief     simulator node source file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-05 updated by Chungwon Kim to use steady clock instead of wall timer
 *            2026-03-05 updated by Chungwon Kim due to addition of actuator node
 *            2026-03-11 updated by Chungwon Kim to add RWA and thruster dynamics
 */

#include "simulation/simulator_node.hpp"

using interfaces::msg::State;
using interfaces::msg::Actuator;

Simulator::Simulator()
    : Node("simulator_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Simulator node...");

    // Declare Parameters
    this->declare_parameter<double>("initial_time", 0.0);
    this->declare_parameter<double>("loop_rate_hz", 100.0);
    this->declare_parameter<std::string>("id", "ego");
    this->declare_parameter<std::string>("frame_id", "world");

    this->declare_parameter<double>("initial_x", 0.0);
    this->declare_parameter<double>("initial_y", 0.0);
    this->declare_parameter<double>("initial_z", 0.0);
    this->declare_parameter<double>("initial_vx", 0.0);
    this->declare_parameter<double>("initial_vy", 0.0);
    this->declare_parameter<double>("initial_vz", 0.0);

    this->declare_parameter<double>("initial_qx", 0.0);
    this->declare_parameter<double>("initial_qy", 0.0);
    this->declare_parameter<double>("initial_qz", 0.0);
    this->declare_parameter<double>("initial_qw", 1.0);
    this->declare_parameter<double>("initial_wx", 0.0);
    this->declare_parameter<double>("initial_wy", 0.0);
    this->declare_parameter<double>("initial_wz", 0.0);

    this->declare_parameter<std::vector<double>>("initial_rwa_momentum",
        std::vector<double>{0.0, 0.0, 0.0, 0.0});

    this->declare_parameter<double>("mass", 1.0);
    this->declare_parameter<double>("inertia_xx", 1.0);
    this->declare_parameter<double>("inertia_yy", 1.0);
    this->declare_parameter<double>("inertia_zz", 1.0);
    this->declare_parameter<double>("inertia_xy", 0.0);
    this->declare_parameter<double>("inertia_xz", 0.0);
    this->declare_parameter<double>("inertia_yz", 0.0);
    this->declare_parameter<std::vector<double>>("center_of_mass",
        std::vector<double>{0.0, 0.0, 0.0});

    this->declare_parameter<double>("max_thrust", 1.0);

    this->declare_parameter<double>("max_rwa_momentum", 0.1);
    this->declare_parameter<double>("max_rwa_torque", 0.01);

    GetParameters();

    RCLCPP_INFO(this->get_logger(),
                "Simulator Parameters: initial_time=%.3f, loop_rate_hz=%.3f, id=%s, frame_id=%s",
                initial_time_, loop_rate_hz_, initial_state_.id.c_str(), initial_state_.header.frame_id.c_str());

    RCLCPP_INFO(this->get_logger(),
                "Initial Linear State: x=%.3f, y=%.3f, z=%.3f, vx=%.3f, vy=%.3f, vz=%.3f",
                initial_state_.pose.position.x, initial_state_.pose.position.y, initial_state_.pose.position.z,
                initial_state_.vel.linear.x, initial_state_.vel.linear.y, initial_state_.vel.linear.z);

    RCLCPP_INFO(this->get_logger(),
                "Initial Angular State: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f, wx=%.3f, wy=%.3f, wz=%.3f",
                initial_state_.pose.orientation.x, initial_state_.pose.orientation.y,
                initial_state_.pose.orientation.z, initial_state_.pose.orientation.w,
                initial_state_.vel.angular.x, initial_state_.vel.angular.y, initial_state_.vel.angular.z);

    RCLCPP_INFO(this->get_logger(),
                "Initial RWA Momentum: rwa_momentum[0]=%.3f, rwa_momentum[1]=%.3f, rwa_momentum[2]=%.3f, rwa_momentum[3]=%.3f",
                initial_state_.rwa_momentum[0], initial_state_.rwa_momentum[1],
                initial_state_.rwa_momentum[2], initial_state_.rwa_momentum[3]);

    RCLCPP_INFO(this->get_logger(),
                "Mass & Inertia: mass=%.3f, Ixx=%.3f, Iyy=%.3f, Izz=%.3f, Ixy=%.3f, Ixz=%.3f, Iyz=%.3f",
                mass_, inertia_(0, 0), inertia_(1, 1), inertia_(2, 2),
                inertia_(0, 1), inertia_(0, 2), inertia_(1, 2));

    RCLCPP_INFO(this->get_logger(),
        "Center of Mass: x=%.3f, y=%.3f, z=%.3f",
        center_of_mass_(0), center_of_mass_(1), center_of_mass_(2));

    RCLCPP_INFO(this->get_logger(),
                "Thruster Limit: max_thrust=%.3f", max_thrust_);

    RCLCPP_INFO(this->get_logger(),
                "RWA Parameters: max_momentum=%.3f, max_torque=%.3f",
                max_rwa_momentum_, max_rwa_torque_);

    // Initialize last command (for safety)
    last_cmd_.id = initial_state_.id;
    last_cmd_.thruster_cmd = std::array<double, 12>{};
    last_cmd_.rwa_torque_cmd = std::array<double, 4>{};

    // QoS settings
    // Publisher QoS
    auto qos_profile_pub = rclcpp::QoS(rclcpp::KeepLast(10),);
    qos_profile_pub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile_pub.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile_pub.deadline(std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)));
    qos_profile_pub.life
    // Subscriber QoS
    auto qos_profile_sub = rclcpp::QoS(rclcpp::KeepLast(1));

    // Subscribers Initialization
    sub_actuator_ = this->create_subscription<Actuator>(
        "actuator", qos_profile_sub,
        std::bind(&Simulator::CallbackActuator, this, std::placeholders::_1));

    // Publishers Initialization
    pub_state_ = this->create_publisher<State>(
        "state", qos_profile_pub);

    // Steady clock initialization
    steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / loop_rate_hz_));

    t_run_node_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        steady_clock_,
        period_ns,
        std::bind(&Simulator::Run, this));

    // Simulator Initialization
    o_state_ = initial_state_;
    sim_time_prev_ = rclcpp::Time(initial_time_);
}

Simulator::~Simulator()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Simulator node...");
}

void Simulator::GetParameters()
{
    // This function can be used to fetch parameters when needed
    this->get_parameter("initial_time", initial_time_);
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
    this->get_parameter("id", initial_state_.id);
    this->get_parameter("frame_id", initial_state_.header.frame_id);

    this->get_parameter("initial_x", initial_state_.pose.position.x);
    this->get_parameter("initial_y", initial_state_.pose.position.y);
    this->get_parameter("initial_z", initial_state_.pose.position.z);
    this->get_parameter("initial_vx", initial_state_.vel.linear.x);
    this->get_parameter("initial_vy", initial_state_.vel.linear.y);
    this->get_parameter("initial_vz", initial_state_.vel.linear.z);

    this->get_parameter("initial_qx", initial_state_.pose.orientation.x);
    this->get_parameter("initial_qy", initial_state_.pose.orientation.y);
    this->get_parameter("initial_qz", initial_state_.pose.orientation.z);
    this->get_parameter("initial_qw", initial_state_.pose.orientation.w);
    this->get_parameter("initial_wx", initial_state_.vel.angular.x);
    this->get_parameter("initial_wy", initial_state_.vel.angular.y);
    this->get_parameter("initial_wz", initial_state_.vel.angular.z);

    std::vector<double> initial_rwa_momentum;
    this->get_parameter("initial_rwa_momentum", initial_rwa_momentum);
    if (initial_rwa_momentum.size() == 4) {
        std::copy_n(initial_rwa_momentum.begin(), 4, initial_state_.rwa_momentum.begin());
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "initial_rwa_momentum size is %zu, expected 4. Using zeros.", initial_rwa_momentum.size());
        initial_state_.rwa_momentum = {0.0, 0.0, 0.0, 0.0};
    }

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

    this->get_parameter("max_thrust", max_thrust_);

    this->get_parameter("max_rwa_momentum", max_rwa_momentum_);
    this->get_parameter("max_rwa_torque", max_rwa_torque_);
}

void Simulator::Init()
{
    // Log
    RCLCPP_INFO(this->get_logger(),
        "Starting Simulator Node Loop with loop_rate_hz=%.3f", loop_rate_hz_);

    // Time Initialization
    o_state_ = initial_state_;
    sim_time_prev_ = rclcpp::Time(initial_time_);

    rclcpp::Time current_time = steady_clock_->now();
    real_time_prev_ = current_time;

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

void Simulator::Run()
{
    if(!sim_initialized_) // first run initializaiton
    {
        Init();
        sim_initialized_ = true;
        o_state_ = initial_state_;
    } else
    {
        // time
        rclcpp::Time current_time = steady_clock_->now();

        // time interval
        double time_dt = (current_time - real_time_prev_).seconds();
        if (time_dt <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Non-positive time step detected: dt=%.6f. Skipping propagation.", time_dt);
            return;
        }
        else
        {
            sim_time_curr_ = sim_time_prev_ + rclcpp::Duration::from_seconds(time_dt);
        }
        real_time_prev_ = current_time;

        // propagate state
        Actuator cmd;
        {
            std::lock_guard<std::mutex> lock(mutex_actuator_);
            cmd = last_cmd_;
        }
        o_state_ = PropagateStateRK4(o_state_, cmd, time_dt);
    }

    // publish state
    pub_state_->publish(o_state_);

    // update sim time
    sim_time_prev_ = sim_time_curr_;
}

StateDerivative Simulator::ComputeStateDerivative(
    const State &state,
    const Actuator &cmd)
{
    // RWA momentum
    Eigen::Vector4d rwa_momentum(
        state.rwa_momentum[0],
        state.rwa_momentum[1],
        state.rwa_momentum[2],
        state.rwa_momentum[3]);

    // actuator commands
    Eigen::Matrix<double, 12, 1> thruster_cmd =
        Eigen::Map<const Eigen::Matrix<double, 12, 1>>(cmd.thruster_cmd.data());
    Eigen::Matrix<double, 4, 1> rwa_cmd =
        Eigen::Map<const Eigen::Matrix<double, 4, 1>>(cmd.rwa_torque_cmd.data());

    // check for actuator limits
    for (int i = 0; i < thruster_cmd.size(); ++i)
    {
        if (thruster_cmd(i) > max_thrust_)
        {
            thruster_cmd(i) = max_thrust_;
        } else if (thruster_cmd(i) < 0.0) {
            thruster_cmd(i) = 0.0;
        }
    }

    for (int i = 0; i < rwa_cmd.size(); ++i)
    {
        if (std::abs(rwa_cmd(i)) > max_rwa_torque_)
        {
            rwa_cmd(i) = std::copysign(max_rwa_torque_, rwa_cmd(i));
        }
    }

    // compute thruster force and torque in body frame
    // construct thruster B matrix
    Eigen::Matrix<double, 3, 12> B_thruster_force = thruster_directions_;
    Eigen::Matrix<double, 3, 12> B_thruster_torque;
    for (int i = 0; i < 12; ++i) {
        B_thruster_torque.col(i) = 
        (thruster_positions_.col(i) - center_of_mass_).cross(thruster_directions_.col(i));
    }

    const Eigen::Vector3d force_body = B_thruster_force * thruster_cmd;
    const Eigen::Vector3d torque_body_thruster = B_thruster_torque * thruster_cmd;

    // compute RWA torque in body frame
    // check for RWA momentum saturation
    for (int i = 0; i < 4; ++i) {
        if (std::abs(rwa_momentum(i)*rwa_cmd(i)) < 0.0) {
            if (abs(rwa_momentum(i)) > max_rwa_momentum_) {
                rwa_cmd(i) = 0.0;
            }
        }
    }
    const Eigen::Vector3d torque_body_rwa = rwa_mounting_matrix_ * rwa_cmd;

    // compute body to inertial DCM
    Eigen::Quaterniond q(state.pose.orientation.w, state.pose.orientation.x,
                         state.pose.orientation.y, state.pose.orientation.z);
    q.normalize();

    const Eigen::Matrix3d D_I2B = q.toRotationMatrix();
    const Eigen::Matrix3d D_B2I = D_I2B.transpose();

    // computing state variable derivatives
    // Kinematics
    StateDerivative derivative{};

    derivative.dx = state.vel.linear.x;
    derivative.dy = state.vel.linear.y;
    derivative.dz = state.vel.linear.z;

    const Eigen::Vector3d w_B(state.vel.angular.x, state.vel.angular.y, state.vel.angular.z);
    const Eigen::Quaterniond omega_b(0.0, w_B.x(), w_B.y(), w_B.z());

    const Eigen::Quaterniond qdot = omega_b * q;

    // Dynamics
    // compute total torque in body frame
    const Eigen::Vector3d torque_body = torque_body_thruster + torque_body_rwa;

    // compute force in inertial frame
    const Eigen::Vector3d force_inertial = D_B2I * force_body;

    const Eigen::Vector3d a_I = force_inertial / mass_;
    derivative.dvx = a_I.x();
    derivative.dvy = a_I.y();
    derivative.dvz = a_I.z();

    derivative.dqx = 0.5 * qdot.x();
    derivative.dqy = 0.5 * qdot.y();
    derivative.dqz = 0.5 * qdot.z();
    derivative.dqw = 0.5 * qdot.w();

    // rotational dynamics (body frame)
    const Eigen::Vector3d h_body = inertia_ * w_B + rwa_mounting_matrix_ * rwa_momentum;
    const Eigen::Vector3d wdot_B = inertia_inv_ * (torque_body - w_B.cross(h_body));

    derivative.dwx = wdot_B.x();
    derivative.dwy = wdot_B.y();
    derivative.dwz = wdot_B.z();

    // RWA momentum derivative
    derivative.drwa_momentum[0] = -rwa_cmd(0);
    derivative.drwa_momentum[1] = -rwa_cmd(1);
    derivative.drwa_momentum[2] = -rwa_cmd(2);
    derivative.drwa_momentum[3] = -rwa_cmd(3);

    return derivative;
}

State Simulator::AddScaledDerivative(
    const State &s,
    const StateDerivative &k,
    const double h) const
{
    // update state, while maintaining the header
    State out = s;

    out.pose.position.x += h * k.dx;
    out.pose.position.y += h * k.dy;
    out.pose.position.z += h * k.dz;

    out.vel.linear.x += h * k.dvx;
    out.vel.linear.y += h * k.dvy;
    out.vel.linear.z += h * k.dvz;

    out.pose.orientation.x += h * k.dqx;
    out.pose.orientation.y += h * k.dqy;
    out.pose.orientation.z += h * k.dqz;
    out.pose.orientation.w += h * k.dqw;

    out.vel.angular.x += h * k.dwx;
    out.vel.angular.y += h * k.dwy;
    out.vel.angular.z += h * k.dwz;

    out.rwa_momentum[0] += h * k.drwa_momentum[0];
    out.rwa_momentum[1] += h * k.drwa_momentum[1];
    out.rwa_momentum[2] += h * k.drwa_momentum[2];
    out.rwa_momentum[3] += h * k.drwa_momentum[3];

    return out;
}

State Simulator::PropagateStateRK4(
    const State &prev,
    const Actuator &cmd,
    const double dt)
{
    // RK4
    const StateDerivative k1 = ComputeStateDerivative(prev, cmd);

    const State s2 = AddScaledDerivative(prev, k1, 0.5 * dt);
    const StateDerivative k2 = ComputeStateDerivative(s2, cmd);

    const State s3 = AddScaledDerivative(prev, k2, 0.5 * dt);
    const StateDerivative k3 = ComputeStateDerivative(s3, cmd);

    const State s4 = AddScaledDerivative(prev, k3, dt);
    const StateDerivative k4 = ComputeStateDerivative(s4, cmd);

    State next = prev;

    next.pose.position.x += (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx);
    next.pose.position.y += (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy);
    next.pose.position.z += (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz);

    next.vel.linear.x += (dt / 6.0) * (k1.dvx + 2.0 * k2.dvx + 2.0 * k3.dvx + k4.dvx);
    next.vel.linear.y += (dt / 6.0) * (k1.dvy + 2.0 * k2.dvy + 2.0 * k3.dvy + k4.dvy);
    next.vel.linear.z += (dt / 6.0) * (k1.dvz + 2.0 * k2.dvz + 2.0 * k3.dvz + k4.dvz);

    next.pose.orientation.x += (dt / 6.0) * (k1.dqx + 2.0 * k2.dqx + 2.0 * k3.dqx + k4.dqx);
    next.pose.orientation.y += (dt / 6.0) * (k1.dqy + 2.0 * k2.dqy + 2.0 * k3.dqy + k4.dqy);
    next.pose.orientation.z += (dt / 6.0) * (k1.dqz + 2.0 * k2.dqz + 2.0 * k3.dqz + k4.dqz);
    next.pose.orientation.w += (dt / 6.0) * (k1.dqw + 2.0 * k2.dqw + 2.0 * k3.dqw + k4.dqw);

    next.vel.angular.x += (dt / 6.0) * (k1.dwx + 2.0 * k2.dwx + 2.0 * k3.dwx + k4.dwx);
    next.vel.angular.y += (dt / 6.0) * (k1.dwy + 2.0 * k2.dwy + 2.0 * k3.dwy + k4.dwy);
    next.vel.angular.z += (dt / 6.0) * (k1.dwz + 2.0 * k2.dwz + 2.0 * k3.dwz + k4.dwz);

    next.rwa_momentum[0] += (dt / 6.0) * (k1.drwa_momentum[0] + 2.0 * k2.drwa_momentum[0] + 2.0 * k3.drwa_momentum[0] + k4.drwa_momentum[0]);
    next.rwa_momentum[1] += (dt / 6.0) * (k1.drwa_momentum[1] + 2.0 * k2.drwa_momentum[1] + 2.0 * k3.drwa_momentum[1] + k4.drwa_momentum[1]);
    next.rwa_momentum[2] += (dt / 6.0) * (k1.drwa_momentum[2] + 2.0 * k2.drwa_momentum[2] + 2.0 * k3.drwa_momentum[2] + k4.drwa_momentum[2]);
    next.rwa_momentum[3] += (dt / 6.0) * (k1.drwa_momentum[3] + 2.0 * k2.drwa_momentum[3] + 2.0 * k3.drwa_momentum[3] + k4.drwa_momentum[3]);

    // quaternion normalization
    {
        Eigen::Quaterniond q(next.pose.orientation.w, next.pose.orientation.x,
                             next.pose.orientation.y, next.pose.orientation.z);
        q.normalize();
        next.pose.orientation.w = q.w();
        next.pose.orientation.x = q.x();
        next.pose.orientation.y = q.y();
        next.pose.orientation.z = q.z();
    }

    // header + sim time
    next.id = prev.id;
    next.header.frame_id = prev.header.frame_id;

    rclcpp::Time sim_time_old = prev.header.stamp;
    rclcpp::Time sim_time_new = sim_time_prev_ + rclcpp::Duration::from_seconds(dt);
    next.header.stamp = sim_time_new;

    return next;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Simulator>());

    rclcpp::shutdown();
    return 0;
}