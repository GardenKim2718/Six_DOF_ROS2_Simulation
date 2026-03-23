/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      display_node.cpp
 * @brief     display node source file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 *            2026-03-05 updated to use steady clock instead of wall timer
 *            2026-03-11 updated to fix TF2 frame orientation
 *            2026-03-23 updated by Chungwon Kim to update QoS policy and add status display
 */

#include "simulation/display_node.hpp"

#include <iomanip>
#include <sstream>

Display::Display(double &loop_rate_hz_)
: Node("display_node")
{
    RCLCPP_INFO(this->get_logger(), "Initialize Display node...");
    
    // TF2 broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Declare Parameters
    this->declare_parameter<double>("loop_rate_hz", 100.0);

    // Read Parameters
    this->get_parameter("loop_rate_hz", loop_rate_hz_);

    // QoS settings
    // Event Callbacks for QoS
    rclcpp::SubscriptionOptions sub_options;
    sub_options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineRequestedInfo & info)
        {
            RCLCPP_WARN(
            this->get_logger(),
            "Subscription deadline missed: total_count=%d, total_count_change=%d",
            info.total_count,
            info.total_count_change);
        };

    // Publisher QoS
    auto qos_profile_pub = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile_pub.reliable();
    qos_profile_pub.transient_local();

    // Subscriber QoS
    auto qos_profile_sub = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile_sub.reliable();
    qos_profile_sub.transient_local();
    qos_profile_sub.deadline(rclcpp::Duration::from_seconds(1.0 / loop_rate_hz_ * 1.2));

    // Create Subscribers
    sub_target_ = this->create_subscription<interfaces::msg::Target>(
        "target", qos_profile_sub,
        std::bind(&Display::CallbackTarget, this, std::placeholders::_1),
        sub_options);
    
    sub_state_ = this->create_subscription<interfaces::msg::State>(
        "state", qos_profile_sub,
        std::bind(&Display::CallbackState, this, std::placeholders::_1),
        sub_options);

    sub_guidance_ = this->create_subscription<interfaces::msg::Guidance>(
        "guidance", qos_profile_sub,
        std::bind(&Display::CallbackGuidance, this, std::placeholders::_1),
        sub_options);

    // Create Publishers
    pub_position_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "position_marker", qos_profile_pub);
    pub_speed_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "speed_marker", qos_profile_pub);
    pub_target_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "target_marker", qos_profile_pub);
    pub_status_text_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "status_text", qos_profile_pub);

    // Initialize offset quaternion for model orientation adjustment
    q_offset.setRPY(M_PI/2.0, 0.0, M_PI/2.0);    // rotate mesh to align with x-forward
    q_offset.normalize();

    // Steady clock initialization
    steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Timer Initialization
    const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / loop_rate_hz_));

    t_run_node_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        steady_clock_,
        period_ns,
        std::bind(&Display::Run, this)
    );
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

    interfaces::msg::Guidance guidance;
    {
        std::lock_guard<std::mutex> lock(mutex_guidance_);
        guidance = last_guidance_;
    }

    // steady-clock time
    rclcpp::Time current_time = this->get_clock()->now();

    // Marker display duration
    const rclcpp::Duration marker_duration = rclcpp::Duration(0, int64_t(1.0/loop_rate_hz_*1e9));

    // Display Target
    if(b_is_target_initialized_){
        DisplayTarget(sim_time_, target, marker_duration);
    }

    // Display State
    if(b_is_sim_initialized_){
        DisplayState(sim_time_, state, marker_duration);
    }

    // Display Status
    if(b_is_sim_initialized_ && b_is_guidance_initialized_) {
        DisplayStatus(sim_time_, state, target, guidance);
    }
}

void Display::DisplayState(const rclcpp::Time& time,
                           const interfaces::msg::State& state,
                           const rclcpp::Duration& duration) {

    // Broadcast TF for the ego frame
    geometry_msgs::msg::TransformStamped tf_state;
    tf_state.header.stamp = time;
    tf_state.header.frame_id = state.header.frame_id;   // e.g., "world"
    tf_state.child_frame_id  = state.id + "_frame";     // e.g., "ego_frame"

    tf_state.transform.translation.x = state.pose.position.x;
    tf_state.transform.translation.y = state.pose.position.y;
    tf_state.transform.translation.z = state.pose.position.z;

    tf_state.transform.rotation.x = state.pose.orientation.x;
    tf_state.transform.rotation.y = state.pose.orientation.y;
    tf_state.transform.rotation.z = state.pose.orientation.z;
    tf_state.transform.rotation.w = state.pose.orientation.w;

    tf_broadcaster_->sendTransform(tf_state);

    // Rendering of spacecraft current state
    visualization_msgs::msg::Marker ego_marker;
    visualization_msgs::msg::Marker speed_marker;

    ego_marker.ns = state.id;
    ego_marker.header.stamp = time;
    ego_marker.header.frame_id = state.id + "_frame";   // ego frame for marker pose
    ego_marker.id = 0;
    ego_marker.lifetime = duration;
    ego_marker.action = visualization_msgs::msg::Marker::ADD;

    ego_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    std::string dir(getenv("PWD"));
    std::string mesh_path("/src/simulation/resources");
    ego_marker.mesh_resource = "file://" + dir + mesh_path + "/X_37.stl";
    ego_marker.mesh_use_embedded_materials = true;

    ego_marker.pose.position.x = 0.0;
    ego_marker.pose.position.y = 0.0;
    ego_marker.pose.position.z = 0.0;
    
    ego_marker.pose.orientation.x = q_offset.x();
    ego_marker.pose.orientation.y = q_offset.y();
    ego_marker.pose.orientation.z = q_offset.z();
    ego_marker.pose.orientation.w = q_offset.w();

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
    speed_marker.header.frame_id = state.id + "_frame";

    speed_marker.ns = state.id + "_speed";
    speed_marker.id = 0;
    speed_marker.action = visualization_msgs::msg::Marker::ADD;
    speed_marker.lifetime = duration;
    speed_marker.type = visualization_msgs::msg::Marker::ARROW;

    // Arrow origin at ego position
    speed_marker.pose.position.x = 0.0;
    speed_marker.pose.position.y = 0.0;
    speed_marker.pose.position.z = 0.0;

    tf2::Vector3 v_inertial(state.vel.linear.x,
                            state.vel.linear.y,
                            state.vel.linear.z);

    tf2::Quaternion q_body(state.pose.orientation.x,
                           state.pose.orientation.y,
                           state.pose.orientation.z,
                           state.pose.orientation.w);

    tf2::Matrix3x3 D_body(q_body);
    tf2::Vector3 v_body = D_body.transpose() * v_inertial;
    
    const double vx = v_body.x();
    const double vy = v_body.y();
    const double vz = v_body.z();
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
                            const interfaces::msg::Target& target,
                            const rclcpp::Duration& duration) {
    visualization_msgs::msg::Marker target_marker;

    // Broadcast TF for the target frame
    geometry_msgs::msg::TransformStamped tf_target;
    tf_target.header.stamp = time;
    tf_target.header.frame_id = target.header.frame_id;   // e.g., "world"
    tf_target.child_frame_id  = target.id + "_target_frame";

    tf_target.transform.translation.x = target.pose.position.x;
    tf_target.transform.translation.y = target.pose.position.y;
    tf_target.transform.translation.z = target.pose.position.z;

    tf_target.transform.rotation.x = target.pose.orientation.x;
    tf_target.transform.rotation.y = target.pose.orientation.y;
    tf_target.transform.rotation.z = target.pose.orientation.z;
    tf_target.transform.rotation.w = target.pose.orientation.w;

    tf_broadcaster_->sendTransform(tf_target);

    //Rendering of target state
    target_marker.ns = target.id + "_target";
    target_marker.header.stamp = time;
    target_marker.header.frame_id = target.id + "_target_frame";
    target_marker.id = 0;
    target_marker.lifetime = duration;
    target_marker.action = visualization_msgs::msg::Marker::ADD;

    // Use mesh instead of sphere
    target_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

    // Build mesh path (same logic as your ego marker)
    std::string dir(getenv("PWD"));
    std::string mesh_path("/src/simulation/resources");
    target_marker.mesh_resource = "file://" + dir + mesh_path + "/X_37.stl";

    // If you want to control color/alpha from the marker, disable embedded materials
    target_marker.mesh_use_embedded_materials = false;

    // Target pose
    target_marker.pose.position.x = 0.0;
    target_marker.pose.position.y = 0.0;
    target_marker.pose.position.z = 0.0;
    
    target_marker.pose.orientation.x = q_offset.x();
    target_marker.pose.orientation.y = q_offset.y();
    target_marker.pose.orientation.z = q_offset.z();
    target_marker.pose.orientation.w = q_offset.w();

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

void Display::DisplayStatus(const rclcpp::Time& time,
                            const interfaces::msg::State& state,
                            const interfaces::msg::Target& target,
                            const interfaces::msg::Guidance& guidance) {
    rviz_2d_overlay_msgs::msg::OverlayText status_text;

    status_text.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    status_text.width = 800;
    status_text.height = 600;
    status_text.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    status_text.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    status_text.horizontal_distance = 10;
    status_text.vertical_distance = 10;
    status_text.text_size = 9.0;
    status_text.line_width = 1;

    status_text.font = "DejaVu Sans Mono";
    status_text.fg_color.r = 0.9f;
    status_text.fg_color.g = 0.9f;
    status_text.fg_color.b = 0.9f;
    status_text.fg_color.a = 0.7f;
    status_text.bg_color.r = 0.0f;
    status_text.bg_color.g = 0.0f;
    status_text.bg_color.b = 0.0f;
    status_text.bg_color.a = 0.0f;

    // guidance status
    bool is_linear_guidance_active = guidance.linear_guidance_active;
    bool is_angular_guidance_active = guidance.angular_guidance_active;

    // status information
    const double vx = state.vel.linear.x;
    const double vy = state.vel.linear.y;
    const double vz = state.vel.linear.z;
    const double velocity = std::sqrt(vx * vx + vy * vy + vz * vz);
    const double wx = state.vel.angular.x;
    const double wy = state.vel.angular.y;
    const double wz = state.vel.angular.z;
    const double ang_speed = std::sqrt(wx * wx + wy * wy + wz * wz);

    // target information
    const double target_x = target.pose.position.x;
    const double target_y = target.pose.position.y;
    const double target_z = target.pose.position.z;
    const double target_vx = target.vel.linear.x;
    const double target_vy = target.vel.linear.y;
    const double target_vz = target.vel.linear.z;
    const double target_qx = target.pose.orientation.x;
    const double target_qy = target.pose.orientation.y;
    const double target_qz = target.pose.orientation.z;
    const double target_qw = target.pose.orientation.w;
    const double target_wx = target.vel.angular.x;
    const double target_wy = target.vel.angular.y;
    const double target_wz = target.vel.angular.z;

    std::ostringstream status_stream;
    status_stream << std::fixed << std::setprecision(3);
    status_stream
        << "Current Time: " << time.seconds() << " s\n\n"
        << "State\n"
        << "  Position [m]      : ("
        << state.pose.position.x << ", "
        << state.pose.position.y << ", "
        << state.pose.position.z << ")\n"
        << "  Speed [m/s]       : ("
        << state.vel.linear.x << ", "
        << state.vel.linear.y << ", "
        << state.vel.linear.z << ")\n"
        << "  Velocity [m/s]     : " << velocity << "\n"
        << "  Quaternion [x y z w]: ("
        << state.pose.orientation.x << ", "
        << state.pose.orientation.y << ", "
        << state.pose.orientation.z << ", "
        << state.pose.orientation.w << ")\n"
        << "  Angular Velocity [rad/s]: ("
        << state.vel.angular.x << ", "
        << state.vel.angular.y << ", "
        << state.vel.angular.z << ")\n"
        << "  Angular Speed [rad/s]: " << ang_speed << "\n"
        << "  RWA Momentum [Nms]: ("
        << state.rwa_momentum[0] << ", "
        << state.rwa_momentum[1] << ", "
        << state.rwa_momentum[2] << ", "
        << state.rwa_momentum[3] << ")\n\n"
        << "Target\n"
        << "  Position [m]      : ("
        << target_x << ", "
        << target_y << ", "
        << target_z << ")\n"
        << "  Speed [m/s]       : ("
        << target_vx << ", "
        << target_vy << ", "
        << target_vz << ")\n"
        << "  Quaternion [x y z w]: ("
        << target_qx << ", "
        << target_qy << ", "
        << target_qz << ", "
        << target_qw << ")\n"
        << "  Angular Velocity [rad/s]: ("
        << target_wx << ", "
        << target_wy << ", "
        << target_wz << ")\n\n"
        << "Guidance\n"
        << "  Linear : " << (is_linear_guidance_active ? "ACTIVE" : "INACTIVE") << "\n"
        << "  Angular: " << (is_angular_guidance_active ? "ACTIVE" : "INACTIVE") << "\n\n";

    status_text.text = status_stream.str();
    pub_status_text_->publish(status_text);
}

int main(int argc, char ** argv)
{
  double loop_rate_hz_ = 100.0;
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Display>(loop_rate_hz_));

  rclcpp::shutdown();
  return 0;
}