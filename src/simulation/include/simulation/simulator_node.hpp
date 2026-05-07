/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      simulator_node.hpp
 * @brief     simulator node header file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-05 updated by Chungwon Kim to use steady clock instead of wall timer
 *            2026-03-05 updated by Chungwon Kim due to addition of actuator node
 *            2026-03-11 updated by Chungwon Kim to add RWA and thruster dynamics
 *            2026-03-27 updated by Chungwon Kim to set subscriber deadline QoS based on FSW loop rate
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
#include "interfaces/msg/actuator.hpp"
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
    double drwa_momentum[4];
};

class Simulator : public rclcpp::Node
{
public:
    Simulator();
    ~Simulator();

    void run();
    void init();
    void get_parameters();

private:
    // Callback function for actuator subscription
    inline void callback_actuator(
        const interfaces::msg::Actuator::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_actuator_);
        last_cmd_ = *msg;
    }

    inline double wrap_to_pi(double angle)
    {
        // wrap to [-pi, pi)
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    StateDerivative compute_state_derivative(
        const interfaces::msg::State &state,
        const interfaces::msg::Actuator &cmd);

    interfaces::msg::State add_scaled_derivative(
        const interfaces::msg::State& s,
        const StateDerivative& k,
        const double h) const;

    interfaces::msg::State propagate_state_rk4(
        const interfaces::msg::State &prev,
        const interfaces::msg::Actuator &cmd,
        const double dt);

    // topics
    rclcpp::Subscription<interfaces::msg::Actuator>::SharedPtr sub_actuator_;
    rclcpp::Publisher<interfaces::msg::State>::SharedPtr pub_state_;  

    // mutex
    std::mutex mutex_actuator_;

    // Steady clock
    rclcpp::Clock::SharedPtr steady_clock_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::Actuator last_cmd_;

    // states
    interfaces::msg::State initial_state_;
    interfaces::msg::State o_state_;

    // time
    double initial_time_{0.0};

    rclcpp::Time sim_time_prev_;
    rclcpp::Time sim_time_curr_;
    rclcpp::Time real_time_prev_;

    // loop rate
    double loop_rate_hz_{100.0};
    const double fsw_loop_rate_hz_{8.0};

    // declare the additional variables for yourself
    bool sim_initialized_{false};

    double mass_{1.0};
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inertia_inv_;
    Eigen::Vector3d center_of_mass_;

    // thruster configuration (12 thrusters)
    Eigen::Matrix<double, 3, 12> thruster_positions_;
    Eigen::Matrix<double, 3, 12> thruster_directions_;
    double max_thrust_{1.0};  // maximum thrust per thruster [N]

    // RWA configuration (4 RWAs)
    Eigen::Matrix<double, 3, 4> rwa_mounting_matrix_;
    double max_rwa_momentum_{0.1};  // maximum momentum storage of each RWA [N*m*s]
    double max_rwa_torque_{0.01};  // maximum torque of each RWA [N*m]
};

#endif  // __simulator_node_hpp__