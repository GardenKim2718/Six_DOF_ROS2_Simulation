/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      actuator_node.hpp
 * @brief     6-DOF actuator node header file
 *
 * @date      2026-03-05 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#ifndef __actuator_node_hpp__
#define __actuator_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include "interfaces/msg/navigation.hpp"
#include "interfaces/msg/command.hpp"
#include "interfaces/msg/actuator.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Actuator : public rclcpp::Node
{
public:
  Actuator();
  ~Actuator();

  void Init();
  void Run();
  void GetParameters();

  private:
    // add your member functions and variables here

    // Callback function for command subscription
    inline void CallbackCommand(
        const interfaces::msg::Command::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_command_);
        last_command_ = *msg;
        b_control_initialized_ = true;
    }

    inline void CallbackNavigation(
        const interfaces::msg::Navigation::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_navigation_);
        last_state_ = *msg;
        b_navigation_initialized_ = true;
    }

    // Custom Functions
    
    // topics
    rclcpp::Publisher<interfaces::msg::Actuator>::SharedPtr pub_actuator_;
    rclcpp::Subscription<interfaces::msg::Command>::SharedPtr sub_command_;  
    rclcpp::Subscription<interfaces::msg::Navigation>::SharedPtr sub_navigation_;

    // mutex
    std::mutex mutex_command_;
    std::mutex mutex_navigation_;

    // Steady clock
    rclcpp::Clock::SharedPtr steady_clock_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::Command last_command_;
    interfaces::msg::Navigation last_state_;

    // output
    interfaces::msg::Actuator o_actuator_;

    // loop rate
    double loop_rate_hz_{20.0};

    // declare additional variables for yourself
    bool b_control_initialized_{false};
    bool b_navigation_initialized_{false};

    bool b_actuator_initialized_{false};

    // Center of Mass Configuration
    Eigen::Vector3d center_of_mass_;

    // thruster configuration (12 thrusters)
    Eigen::Matrix<double, 3, 12> thruster_positions_;
    Eigen::Matrix<double, 3, 12> thruster_directions_;
    double max_thrust_{1.0};  // maximum thrust per thruster [N]

    // RWA configuration (4 RWAs)
    Eigen::Matrix<double, 3, 4> rwa_mounting_matrix_;
    Eigen::Matrix<double, 4, 3> rwa_mounting_matrix_pseudo_inverse_;
    double max_rwa_momentum_{0.1};  // maximum momentum storage of each RWA [N*m*s]
    double max_rwa_torque_{0.01};  // maximum torque of each RWA [N*m]
};

#endif  // __actuator_node_hpp__