/**
 * @copyright KAIST, Department of Aerospace Engineering, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      navigation_node.hpp
 * @brief     6-DOF navigation node header file
 *
 * @date      2026-03-05 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-11 updated by Chungwon Kim due to addition of actuators
 */

#ifndef __navigation_node_hpp__
#define __navigation_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include "interfaces/msg/state.hpp"
#include "interfaces/msg/navigation.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Navigation : public rclcpp::Node
{
public:
  Navigation();
  ~Navigation();

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

    // Custom Functions
    
    // topics
    rclcpp::Publisher<interfaces::msg::Navigation>::SharedPtr pub_navigation_;
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr sub_state_;  

    // mutex
    std::mutex mutex_state_;

    // Steady clock
    rclcpp::Clock::SharedPtr steady_clock_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // input
    interfaces::msg::State last_state_;

    // output
    interfaces::msg::Navigation o_navigation_;

    // loop rate
    double loop_rate_hz_{20.0};

    // declare additional variables for yourself
    bool b_simulator_initialized_{false};
    bool b_navigation_initialized_{false};
};

#endif  // __navigation_node_hpp__