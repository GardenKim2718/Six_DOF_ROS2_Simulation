/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      display_node.cpp
 * @brief     display node source file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#include "simulation/display_node.hpp"

Display::Display(double &loop_rate_hz_)
: Node("display_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Display node...");
    
    //QoS settings
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Declare Parameters
    this->declare_parameter<double>("loop_rate_hz", 100.0);

    // Read Parameters
    this->get_parameter("loop_rate_hz", loop_rate_hz_);

    // Create Subscribers
    sub_target_ = this->create_subscription<interfaces::msg::Target>(
        "target2d", qos_profile,
        std::bind(&Display::CallbackTarget, this, std::placeholders::_1));
    
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state2d", qos_profile,
        std::bind(&Display::CallbackState, this, std::placeholders::_1));

    // Create Publishers
    pub_position_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "position_marker", qos_profile);
    pub_speed_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "speed_marker", qos_profile);
    pub_target_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "target_marker", qos_profile);

    // Initialize offset quaternion for model orientation adjustment
    q_offset.setRPY(M_PI/2.0, 0.0, 0.0);    // rotate mesh to align with x-forward
    q_offset.normalize();

    // Create Timer
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });
}

Display::~Display()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Display node...");
}

void Display::Run()
{
    // get subscribed data
    interfaces::msg::State state;
    {
        std::lock_guard<std::mutex> lock(mutex_state_);
        if(!b_is_sim_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "No state received yet. Skipping display update.");
            return;
        }
        state = last_state_;
        sim_time_ = state.header.stamp;
    }

    interfaces::msg::Target target; 
    {
        std::lock_guard<std::mutex> lock(mutex_target_);
        target = last_target_;
    }

    // Display Target
    if(b_is_target_initialized_){
        DisplayTarget(sim_time_, target);
    }

    // Display State
    if(b_is_sim_initialized_){
        DisplayState(sim_time_, state);
    }
}

void Display::DisplayState(const rclcpp::Time& time,
                           const interfaces::msg::State& state) {
    visualization_msgs::msg::Marker ego_marker;
    visualization_msgs::msg::Marker speed_marker;

    ego_marker.ns = state.id;
    ego_marker.header.stamp = time;
    ego_marker.header.frame_id = state.header.frame_id;
    ego_marker.id = 0;
    ego_marker.lifetime = rclcpp::Duration(0, int64_t(1.0*1e9)); // 1.0 sec
    ego_marker.action = visualization_msgs::msg::Marker::ADD;

    ego_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    std::string dir(getenv("PWD"));
    std::string mesh_path("/src/simple_simulation_rviz/resources");
    ego_marker.mesh_resource = "file://" + dir + mesh_path + "/Space_core.stl";
    ego_marker.mesh_use_embedded_materials = true;

    ego_marker.pose.position.x = state.pose.position.x;
    ego_marker.pose.position.y = state.pose.position.y;
    ego_marker.pose.position.z = state.pose.position.z;

    tf2::Quaternion q_state(
        state.pose.orientation.x,
        state.pose.orientation.y,
        state.pose.orientation.z,
        state.pose.orientation.w);
    
    tf2::Quaternion q_marker = q_state * q_offset;      // apply offset rotation of the model
    q_marker.normalize();
    ego_marker.pose.orientation.x = q_marker.x();
    ego_marker.pose.orientation.y = q_marker.y();
    ego_marker.pose.orientation.z = q_marker.z();
    ego_marker.pose.orientation.w = q_marker.w();
    
    ego_marker.scale.x = 0.001;
    ego_marker.scale.y = 0.001;
    ego_marker.scale.z = 0.001;

    ego_marker.color.a = 1.0;
    ego_marker.color.r = 1.0;
    ego_marker.color.g = 1.0;
    ego_marker.color.b = 1.0;

    pub_position_marker_->publish(ego_marker);

    // Speed Arrow Marker
    speed_marker.header.stamp = time;
    speed_marker.header.frame_id = state.header.frame_id;

    speed_marker.ns = state.id + "_speed";
    speed_marker.id = 0;
    speed_marker.action = visualization_msgs::msg::Marker::ADD;
    speed_marker.lifetime = rclcpp::Duration(0, int64_t(1.0*1e9)); // 1.0 sec
    speed_marker.type = visualization_msgs::msg::Marker::ARROW;

    // Arrow origin at ego position
    speed_marker.pose.position.x = state.pose.position.x;
    speed_marker.pose.position.y = state.pose.position.y;
    speed_marker.pose.position.z = state.pose.position.z;

    const double vx = state.vel.linear.x;
    const double vy = state.vel.linear.y;
    const double vz = state.vel.linear.z;
    const double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

    tf2::Quaternion q_vel;
    if (speed < 1e-6) {
        // set to no rotation if speed is too small (numerical instability)
        q_vel.setRPY(0.0, 0.0, 0.0);
    } else {
        const tf2::Vector3 vel_norm = tf2::Vector3(vx, vy, vz).normalize();
        const tf2::Vector3 x_axis = tf2::Vector3(1.0, 0.0, 0.0);

        tf2::Vector3 axis = x_axis.cross(vel_norm);
        const double axis_norm = axis.length();
        const double dot = std::max(-1.0, std::min(1.0, x_axis.dot(vel_norm)));
        const double angle = std::acos(dot);
        
        if (axis_norm < 1e-9) {
            if(dot > 0.0) {
                // no rotation
                q_vel.setRPY(0.0, 0.0, 0.0);
            } else {
                // 180 degree rotation around any axis perpendicular to x_axis
                q_vel.setRPY(0.0, M_PI, 0.0); // for example, rotate around y-axis
            }
        } else {
            axis /= axis_norm; // normalize axis
            q_vel.setRotation(axis, angle);
        }
    }
    q_vel.normalize();

    speed_marker.pose.orientation.x = q_vel.x();
    speed_marker.pose.orientation.y = q_vel.y();
    speed_marker.pose.orientation.z = q_vel.z();
    speed_marker.pose.orientation.w = q_vel.w();

    // Arrow scale based on speed
    const double min_arrow_len = 0.01;
    const double scale_gain     = 1.0;  // arrow length per m/s
    speed_marker.scale.x = std::max(min_arrow_len, scale_gain * speed);
    speed_marker.scale.y = 0.05;
    speed_marker.scale.z = 0.05;

    // Color: Blue
    speed_marker.color.a = 1.0;
    speed_marker.color.r = 0.0;
    speed_marker.color.g = 0.0;
    speed_marker.color.b = 1.0;

    pub_speed_marker_->publish(speed_marker);
}

void Display::DisplayTarget(const rclcpp::Time& time,
                            const interfaces::msg::Target& target) {
    visualization_msgs::msg::Marker target_marker;

    target_marker.ns = target.id + "_target";
    target_marker.header.stamp = time;
    target_marker.header.frame_id = target.header.frame_id;
    target_marker.id = 0;
    target_marker.lifetime = rclcpp::Duration(0, int64_t(1.0*1e9)); // 1.0 sec
    target_marker.action = visualization_msgs::msg::Marker::ADD;

    // Use mesh instead of sphere
    target_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

    // Build mesh path (same logic as your ego marker)
    std::string dir(getenv("PWD"));
    std::string mesh_path("/src/simple_simulation_rviz/resources");
    target_marker.mesh_resource = "file://" + dir + mesh_path + "/Space_core.stl";

    // If you want to control color/alpha from the marker, disable embedded materials
    target_marker.mesh_use_embedded_materials = false;

    // Target pose
    target_marker.pose.position.x = target.pose.position.x;
    target_marker.pose.position.y = target.pose.position.y;
    target_marker.pose.position.z = target.pose.position.z;

    // Target attitude
    tf2::Quaternion q_target(
        target.pose.orientation.x,
        target.pose.orientation.y,
        target.pose.orientation.z,
        target.pose.orientation.w);
    
    tf2::Quaternion q_marker = q_target * q_offset;      // apply offset rotation of the model
    q_marker.normalize();
    target_marker.pose.orientation.x = q_marker.x();
    target_marker.pose.orientation.y = q_marker.y();
    target_marker.pose.orientation.z = q_marker.z();
    target_marker.pose.orientation.w = q_marker.w();

    // marker scale
    target_marker.scale.x = 0.001;
    target_marker.scale.y = 0.001;
    target_marker.scale.z = 0.001;

    // Semi-transparent color
    target_marker.color.a = 0.4;
    target_marker.color.r = 0.0;
    target_marker.color.g = 0.3;
    target_marker.color.b = 1.0;

    pub_target_marker_->publish(target_marker);
}

int main(int argc, char ** argv)
{
  double loop_rate_hz_ = 100.0;
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Display>(loop_rate_hz_));

  rclcpp::shutdown();
  return 0;
}