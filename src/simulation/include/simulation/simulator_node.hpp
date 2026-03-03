/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      simulator_node.hpp
 * @brief     simulator node header file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#ifndef __simulator_node_hpp__
#define __simulator_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <array>
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

struct StateDerivative
{
    double dx;
    double dy;
    double dz;
    double dvx;
    double dvy;
    double dvz;
    double dqx;
    double dqy;
    double dqz;
    double dqw;
    double dwx;
    double dwy;
    double dwz;
};

class Simulator : public rclcpp::Node
{
public:
    Simulator();
    ~Simulator();

    void Run();
    inline interfaces::msg::State GetInitialState() const { return o_initial_state_; };
    void GetParameters();

private:
    // Steady clock
    rclcpp::Clock steady_clock{RCL_STEADY_TIME};

    // Callback function for command subscription
    inline void CallbackCommand(
        const interfaces::msg::Command::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_command_);
        last_cmd_ = *msg;
    }

    inline double wrapToPi(double angle)
    {
        // wrap to [-pi, pi)
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    StateDerivative ComputeStateDerivative(
        const interfaces::msg::State &state,
        const interfaces::msg::Command &cmd);

    interfaces::msg::State AddScaledDerivative(
        const interfaces::msg::State& s,
        const StateDerivative& k,
        const double h) const;

    interfaces::msg::State PropagateStateRK4(
        const interfaces::msg::State &prev,
        const interfaces::msg::Command &cmd,
        const double dt);

    // topics
    rclcpp::Subscription<interfaces::msg::Command>::SharedPtr sub_command_;
    rclcpp::Publisher<interfaces::msg::State>::SharedPtr pub_state_;  

    // mutex
    std::mutex mutex_command_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::Command last_cmd_;

    // states
    interfaces::msg::State o_initial_state_;
    interfaces::msg::State o_state_;

    // time
    double initial_time_{0.0};

    rclcpp::Time sim_time_prev_;
    rclcpp::Time sim_time_curr_;
    rclcpp::Time real_time_prev_;

    // loop rate
    double loop_rate_hz_{100.0};

    // declare the additional variables for yourself
    std::string frame_id_ = "world";

    double mass_{1.0};
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inertia_inv_;

    double max_force_{10.0};
    double max_torque_{10.0};
};

#endif  // __simulator_node_hpp__