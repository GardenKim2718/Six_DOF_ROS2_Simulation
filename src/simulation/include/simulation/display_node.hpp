/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      display_node.hpp
 * @brief     display node header file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#define __display_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "interfaces/msg/state.hpp"
#include "interfaces/msg/guidance.hpp"
#include "interfaces/msg/target.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Display : public rclcpp::Node {
    public :
        explicit Display(double &loop_rate_hz_);
        ~Display();

        void Init();
        void Run();
        void UpdateParameters();
    
    private :

    // Callback function for state subscription
    inline void CallbackState(
        const interfaces::msg::State::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        last_state_ = *msg;
        b_is_sim_initialized_ = true;
    }

    // Callback function for target state subscription
    inline void CallbackTarget(
        const interfaces::msg::Target::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_target_);
        last_target_ = *msg;
        b_is_target_initialized_ = true;
    }

    void DisplayState(const rclcpp::Time& time, const interfaces::msg::State& state);
    void DisplayTarget(const rclcpp::Time& time, const interfaces::msg::Target& target);

    // subscriber topics
    rclcpp::Subscription<interfaces::msg::Target>::SharedPtr sub_target_;
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr sub_state_;

    // publisher topics
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_position_marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_speed_marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_target_marker_;

    // mutex
    std::mutex mutex_state_;
    std::mutex mutex_target_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // bool
    bool b_is_sim_initialized_ = false;
    bool b_is_target_initialized_ = false;

    // inputs
    interfaces::msg::State   last_state_;
    interfaces::msg::Target  last_target_;
    
    // variables
    rclcpp::Time sim_time_;

    tf2::Quaternion q_offset;
};