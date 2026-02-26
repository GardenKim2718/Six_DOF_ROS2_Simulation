/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      guidance_node.hpp
 * @brief     6-DOF guidance node header file
 *
 * @date      2026-02-26 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */
#ifndef __guidance_node_hpp__
#define __guidance_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include "interfaces/msg/state.hpp"
#include "interfaces/msg/guidance.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Guidance : public rclcpp::Node
{
public:
  Guidance();
  ~Guidance();

  void Init(const interfaces::msg::State& initial_state);
  void Run();
  void GetParameters();

  private:
    // add your member functions and variables here

    // Callback function for command subscription
    inline void CallbackState(
        const interfaces::msg::State::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        last_state_ = *msg;
        b_simulator_initialized_ = true;
    }

    // Custom Functions
    inline Eigen::Quaterniond QuaternionConjugate(
        const Eigen::Quaterniond& q)
    {
        // return conjugate of quaternion
        return Eigen::Quaterniond(q.w(), -q.x(), -q.y(), -q.z());
    }

    inline Eigen::Quaterniond QuaternionSignCorrection(
        const Eigen::Quaterniond& q)
    {
        // Ensure that the quaternion scalar part is non-negative
        if (q.w() < 0.0)
        {
            return Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
        } else {
            return q;
        }    
    }

    // topics
    rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub_guidance_;
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr sub_state_;  

    // mutex
    std::mutex mutex_state_;

    // Steady clock
    rclcpp::Clock steady_clock{RCL_STEADY_TIME};

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::State last_state_;

    // output
    interfaces::msg::Command o_guidance_;

    // time
    double time_prev_{0.0};

    // loop rate
    double loop_rate_hz_{20.0};

    // target state
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double target_z_ = 0.0;
    double target_qx_ = 0.0;
    double target_qy_ = 0.0;
    double target_qz_ = 0.0;
    double target_qw_ = 1.0;

    // dynamic parameters
    double mass_ = 1.0;
    Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d inertia_inv_ = Eigen::Matrix3d::Identity();

    // hardware limits
    double max_force_ = 10.0;    // maximum force [N]
    double max_torque_ = 10.0;   // maximum torque [N*m]

    // declare the additional variables for yourself
    bool b_simulator_initialized_ = false;
    bool b_guidance_initialized_ = false;

    rclcpp::Time sim_time_prev_;
    rclcpp::Time sim_time_curr_;

    double time_dt_ = 0.0;

    Eigen::Vector3d err_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d err_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_err_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d integral_err_pos_ = Eigen::Vector3d::Zero();

    Eigen::Quaterniond quat_curr_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quat_des_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond err_quat_ = Eigen::Quaterniond::Identity();

    Eigen::Vector3d eigen_vec_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_curr_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_des_ = Eigen::Vector3d::Zero();
};

#endif  // __guidance_node_hpp__