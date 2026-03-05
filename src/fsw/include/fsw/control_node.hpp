/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      control_node.hpp
 * @brief     6-DOF control node header file
 *
 * @date      2026-02-09 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-02-13 expanded by Chungwon Kim for 6-DOF control
 *            2026-03-03 edited by Chungwon Kim for Guidance-Command interface update
 *            2026-03-05 updated to use steady clock instead of wall timer
 */

#ifndef __control_node_hpp__
#define __control_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include "interfaces/msg/state.hpp"
#include "interfaces/msg/command.hpp"
#include "interfaces/msg/guidance.hpp"
#include "interfaces/msg/target.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Control : public rclcpp::Node
{
public:
  Control();
  ~Control();

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

    inline void CallbackTarget(
        const interfaces::msg::Target::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_target_);
        last_target_ = *msg;
        b_target_initialized_ = true;
    }

    inline void CallbackGuidance(
        const interfaces::msg::Guidance::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_guidance_);
        last_guidance_ = *msg;
        b_guidance_initialized_ = true;
    }

    // Custom Functions
    inline Eigen::Quaterniond QuaternionConjugate(
        const Eigen::Quaterniond &q)
    {
        // return conjugate of quaternion
        return Eigen::Quaterniond(q.w(), -q.x(), -q.y(), -q.z());
    }

    inline Eigen::Quaterniond QuaternionSignCorrection(
        const Eigen::Quaterniond &q)
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
    rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub_command_;
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr sub_state_;  
    rclcpp::Subscription<interfaces::msg::Guidance>::SharedPtr sub_guidance_;
    rclcpp::Subscription<interfaces::msg::Target>::SharedPtr sub_target_;

    // mutex
    std::mutex mutex_state_;
    std::mutex mutex_guidance_;
    std::mutex mutex_target_;

    // Steady clock
    rclcpp::Clock::SharedPtr steady_clock_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::State last_state_;
    interfaces::msg::Guidance last_guidance_;
    interfaces::msg::Target last_target_;

    // output
    interfaces::msg::Command o_command_;

    // time
    double time_prev_{0.0};

    // loop rate
    double loop_rate_hz_{20.0};

    // control gains
    double linear_kp_ = 1.0;
    double linear_ki_ = 0.0;
    double linear_kd_ = 0.0;
    double angular_kp_ = 1.0;
    double angular_ki_ = 0.0;
    double angular_kd_ = 0.0;

    // dynamic parameters
    double mass_ = 1.0;
    Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d inertia_inv_ = Eigen::Matrix3d::Identity();

    // hardware limits
    double max_force_ = 10.0;    // maximum force [N]
    double max_torque_ = 10.0;   // maximum torque [N*m]

    // declare the additional variables for yourself
    bool b_linear_guidance_active_ = false;
    bool b_angular_guidance_active_ = false;

    bool b_simulator_initialized_ = false;
    bool b_guidance_initialized_ = false;
    bool b_target_initialized_ = false;
    bool b_control_initialized_ = false;

    rclcpp::Time sim_time_prev_;
    rclcpp::Time sim_time_curr_;

    double time_dt_ = 0.0;

    Eigen::Vector3d err_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d err_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_err_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d integral_err_pos_ = Eigen::Vector3d::Zero();

    Eigen::Quaterniond err_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond prev_err_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d err_ang_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d integral_err_quat_ = Eigen::Vector3d::Zero();   // error quaternion vector part integration
};

#endif  // __control_node_hpp__