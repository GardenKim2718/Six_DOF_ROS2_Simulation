/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      display_node.hpp
 * @brief     display node header file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-05 updated to use steady clock instead of wall timer
 *            2026-03-23 updated by Chungwon Kim to update QoS policy and add status display
 */

#define __display_node_hpp__

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include <visualization_msgs/msg/marker.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

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

        void init();
        void run();
        void UpdateParameters();
    
    private :
    // Callback function for state subscription
    inline void callback_state(
        const interfaces::msg::State::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        last_state_ = *msg;
        b_is_sim_initialized_ = true;
    }

    // Callback function for target state subscription
    inline void callback_target(
        const interfaces::msg::Target::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_target_);
        last_target_ = *msg;
        b_is_target_initialized_ = true;
    }

    // Callback function for guidance subscription
    inline void callback_guidance(
        const interfaces::msg::Guidance::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_guidance_);
        last_guidance_ = *msg;
        b_is_guidance_initialized_ = true;
    }

    void DisplayState(const rclcpp::Time& time, const interfaces::msg::State& state,
                      const rclcpp::Duration& duration);
                      
    void DisplayTarget(const rclcpp::Time& time, const interfaces::msg::Target& target,
                       const rclcpp::Duration& duration);

    void DisplayStatus(const rclcpp::Time& time,
                       const interfaces::msg::State& state,
                       const interfaces::msg::Target& target,
                       const interfaces::msg::Guidance& guidance);

    // subscriber topics
    rclcpp::Subscription<interfaces::msg::Target>::SharedPtr sub_target_;
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr sub_state_;
    rclcpp::Subscription<interfaces::msg::Guidance>::SharedPtr sub_guidance_;

    // publisher topics
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_position_marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_speed_marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_target_marker_;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr pub_status_text_;

    // TF2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // mutex
    std::mutex mutex_state_;
    std::mutex mutex_target_;
    std::mutex mutex_guidance_;

    // Steady clock
    rclcpp::Clock::SharedPtr steady_clock_;

    // timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // loop rate
    double loop_rate_hz_;

    // bool
    bool b_is_sim_initialized_ = false;
    bool b_is_target_initialized_ = false;
    bool b_is_guidance_initialized_ = false;

    // inputs
    interfaces::msg::State   last_state_;
    interfaces::msg::Target  last_target_;
    interfaces::msg::Guidance last_guidance_;

    // variables
    rclcpp::Time sim_time_;

    tf2::Quaternion q_offset;
};