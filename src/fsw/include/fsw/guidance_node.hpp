/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      guidance_node.hpp
 * @brief     6-DOF guidance node header file
 *
 * @date      2026-02-26 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-01 edited by Chungwon Kim (added linear guidance logic based on Apollo Powered Descent Guidance)
 *            2026-03-03 edited by Chungwon Kim (added rotational guidance logic based on Apollo Powered Descent Guidance application on attitude guidance)
 *            2026-03-05 updated by Chungwon Kim to use steady clock instead of wall timer
 *            2026-03-05 updated by Chungwon Kim due to addition of navigation
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

#include "interfaces/msg/navigation.hpp"
#include "interfaces/msg/guidance.hpp"
#include "interfaces/msg/target.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Guidance : public rclcpp::Node
{
public:
  Guidance();
  ~Guidance();

  void Init(const interfaces::msg::Navigation& initial_state);
  void Run();
  void GetParameters();

  private:
    // add your member functions and variables here

    // Callback function for command subscription
    inline void CallbackNavigation(
        const interfaces::msg::Navigation::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_navigation_);
        last_state_ = *msg;
        b_navigation_initialized_ = true;
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

    void LinearGuidance(
        const Eigen::Vector3d &x0, const Eigen::Vector3d &xf,
        const Eigen::Vector3d &v0, const Eigen::Vector3d &vf,
        double& T_go_linear_, Eigen::Vector3d& accel_cmd_);
    
    void FindTimeToGoLinear(
        const Eigen::Vector3d &x0, const Eigen::Vector3d &xf,
        const Eigen::Vector3d &v0, const Eigen::Vector3d &vf,
        const double acc_limit);

    bool ApolloPoweredDescentGuidanceValidate(
        const Eigen::Vector3d &x0, const Eigen::Vector3d &xf,
        const Eigen::Vector3d &v0, const Eigen::Vector3d &vf,
        const double T_go, const double acc_limit);

    void AngularGuidance(
        const Eigen::Quaterniond err_quat,
        const Eigen::Vector3d curr_ang_speed, const Eigen::Vector3d target_ang_vel,
        double &T_go_angular_, const double ang_acc_limit_,
        Eigen::Vector3d &ang_accel_cmd_, bool &b_angular_guidance_active_);
    
    // topics
    rclcpp::Publisher<interfaces::msg::Guidance>::SharedPtr pub_guidance_;
    rclcpp::Publisher<interfaces::msg::Target>::SharedPtr pub_target_;
    rclcpp::Subscription<interfaces::msg::Navigation>::SharedPtr sub_navigation_;  

    // mutex
    std::mutex mutex_navigation_;

    // Steady clock
    rclcpp::Clock::SharedPtr steady_clock_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::Navigation last_state_;

    // output
    interfaces::msg::Guidance o_guidance_;
    interfaces::msg::Target o_target_;

    // time
    double time_prev_{0.0};

    // loop rate
    double loop_rate_hz_{20.0};

    // target state
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double target_z_ = 0.0;

    double target_vx_ = 0.0;
    double target_vy_ = 0.0;
    double target_vz_ = 0.0;

    double target_qx_ = 0.0;
    double target_qy_ = 0.0;
    double target_qz_ = 0.0;
    double target_qw_ = 1.0;

    double target_wx_ = 0.0;
    double target_wy_ = 0.0;
    double target_wz_ = 0.0;

    // dynamic parameters
    double mass_ = 1.0;
    Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d inertia_inv_ = Eigen::Matrix3d::Identity();

    // hardware limits
    double max_force_ = 10.0;    // maximum force [N]
    double max_torque_ = 10.0;   // maximum torque [N*m]

    // guidance parameters
    double T_go_linear_ = 10.0;   // time-to-go for linear guidance [s]
    double T_go_linear_min_ = 0.5;    // minimum time-to-go for linear guidance [s]
    double T_go_angular_ = 5.0;   // time-to-go for rotational guidance [s]
    double T_go_angular_min_ = 1.0;   // minimum time-to-go for rotational guidance [s]

    double angular_kp_ = 1.0;   // proportional gain for angular guidance
    double angular_kd_ = 1.0;   // derivative gain for angular guidance

    // declare additional variables for yourself
    bool b_navigation_initialized_ = false;
    bool b_guidance_initialized_ = false;
    
    bool b_linear_guidance_initialized_ = false;
    bool b_angular_guidance_initialized_ = false;

    bool b_linear_guidance_active_ = true;
    bool b_angular_guidance_active_ = true;

    rclcpp::Time sim_time_prev_;
    rclcpp::Time sim_time_curr_;

    double time_dt_ = 0.0;

    Eigen::Vector3d target_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_speed_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d target_ang_vel_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d curr_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d curr_speed_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond curr_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d curr_ang_speed_ = Eigen::Vector3d::Zero();

    Eigen::Quaterniond quat_curr_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quat_des_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond err_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d err_ang_vel_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d eigen_vec_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_curr_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_des_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d accel_cmd_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_accel_cmd_ = Eigen::Vector3d::Zero();
};

#endif  // __guidance_node_hpp__