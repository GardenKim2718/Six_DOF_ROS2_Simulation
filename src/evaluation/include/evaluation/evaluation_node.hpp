/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      evaluation_node.hpp
 * @brief     6-DOF evaluation node header file
 *
 * @date      2026-03-12 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#ifndef __evaluation_node_hpp__
#define __evaluation_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include "interfaces/msg/state.hpp"
#include "interfaces/msg/actuator.hpp"
#include "interfaces/msg/command.hpp"
#include "interfaces/msg/guidance.hpp"
#include "interfaces/msg/target.hpp"
#include "interfaces/msg/evaluation.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Evaluation : public rclcpp::Node
{
public:
  Evaluation();
  ~Evaluation();

  void init();
  void run();
  void get_parameters();

  private:
    // add your member functions and variables here

    // Callback function for command subscription
    inline void callback_state(
        const interfaces::msg::State::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        last_state_ = *msg;
        b_simulator_initialized_ = true;
    }

    inline void callback_actuator(
        const interfaces::msg::Actuator::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_actuator_);
        last_actuator_ = *msg;
        b_actuator_initialized_ = true;
    }

    inline void callback_command(
        const interfaces::msg::Command::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_command_);
        last_command_ = *msg;
        b_control_initialized_ = true;
    }

    inline void callback_guidance(
        const interfaces::msg::Guidance::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_guidance_);
        last_guidance_ = *msg;
        b_guidance_initialized_ = true;
    }

    inline void callback_target(
        const interfaces::msg::Target::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_target_);
        last_target_ = *msg;
        b_target_initialized_ = true;
    }

    // Custom Functions
    
    // topics
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr sub_state_;
    rclcpp::Subscription<interfaces::msg::Actuator>::SharedPtr sub_actuator_;
    rclcpp::Subscription<interfaces::msg::Command>::SharedPtr sub_command_;
    rclcpp::Subscription<interfaces::msg::Guidance>::SharedPtr sub_guidance_;
    rclcpp::Subscription<interfaces::msg::Target>::SharedPtr sub_target_;

    rclcpp::Publisher<interfaces::msg::Evaluation>::SharedPtr pub_evaluation_;

    // mutex
    std::mutex mutex_state_;
    std::mutex mutex_actuator_;
    std::mutex mutex_command_;
    std::mutex mutex_guidance_;
    std::mutex mutex_target_;

    // Steady clock
    rclcpp::Clock::SharedPtr steady_clock_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::State last_state_;
    interfaces::msg::Actuator last_actuator_;
    interfaces::msg::Command last_command_;
    interfaces::msg::Guidance last_guidance_;
    interfaces::msg::Target last_target_;

    // output
    interfaces::msg::Evaluation o_evaluation_;

    // loop rate
    double loop_rate_hz_{20.0};

    // declare additional variables for yourself
    bool b_simulator_initialized_{false};
    bool b_guidance_initialized_{false};
    bool b_control_initialized_{false};
    bool b_actuator_initialized_{false};
    bool b_target_initialized_{false};
    bool b_evaluation_initialized_{false};

    // RWA
    Eigen::Matrix<double, 3, 4> rwa_mounting_matrix_;

    // Thruster
    Eigen::Matrix<double, 3, 12> thruster_positions_;
    Eigen::Matrix<double, 3, 12> thruster_directions_;

    // Dynamics
    double mass_;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inertia_inv_;
    Eigen::Vector3d center_of_mass_;
};

#endif  // __evaluation_node_hpp__