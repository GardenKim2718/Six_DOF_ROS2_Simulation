/**
 * @copyright KAIST, Department of Aerospace Engineering, Spaceflight Mission & Control Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      simulator_node.cpp
 * @brief     simulator node source file
 *
 * @date      2026-02-11 created by Chungwon Kim (gardenkim@kaist.ac.kr)
 */

#include "simulation/simulator_node.hpp"

using interfaces::msg::State;
using interfaces::msg::Command;

StateDerivative Simulator::ComputeStateDerivative(
    const State &state,
    const Command &cmd)
{
  StateDerivative derivative{};

  derivative.dx = state.vel.linear.x;
  derivative.dy = state.vel.linear.y;
  derivative.dz = state.vel.linear.z;

  // compute body to inertial DCM
  Eigen::Quaterniond q(state.pose.orientation.w, state.pose.orientation.x,
                       state.pose.orientation.y, state.pose.orientation.z);
  q.normalize();

  const Eigen::Matrix3d D_I2B = q.toRotationMatrix();
  const Eigen::Matrix3d D_B2I = D_I2B.transpose();

  // check for force/torque limits
  Eigen::Vector3d force_cmd(cmd.force.x, cmd.force.y, cmd.force.z);
  Eigen::Vector3d torque_cmd(cmd.torque.x, cmd.torque.y, cmd.torque.z);

  if (force_cmd.norm() > max_force_) {
    force_cmd = (force_cmd / force_cmd.norm()) * max_force_;
  }
  if (torque_cmd.norm() > max_torque_) {
    torque_cmd = (torque_cmd / torque_cmd.norm()) * max_torque_;
  }

  // compute force in inertial frame
  const Eigen::Vector3d F_B(force_cmd.x(), force_cmd.y(), force_cmd.z());
  const Eigen::Vector3d F_I = D_B2I * F_B;

  const Eigen::Vector3d a_I = F_I / mass_;
  derivative.dvx = a_I.x();
  derivative.dvy = a_I.y();
  derivative.dvz = a_I.z();

  const Eigen::Vector3d w_B(state.vel.angular.x, state.vel.angular.y, state.vel.angular.z);
  const Eigen::Quaterniond omega_b(0.0, w_B.x(), w_B.y(), w_B.z());

  const Eigen::Quaterniond qdot = omega_b * q;

  derivative.dqx = 0.5 * qdot.x();
  derivative.dqy = 0.5 * qdot.y();
  derivative.dqz = 0.5 * qdot.z();
  derivative.dqw = 0.5 * qdot.w();

  // rotational dynamics (body frame)
  const Eigen::Vector3d tau_B(torque_cmd.x(), torque_cmd.y(), torque_cmd.z());

  const Eigen::Vector3d Iw = inertia_ * w_B;
  const Eigen::Vector3d wdot_B = inertia_inv_ * (tau_B - w_B.cross(Iw));

  derivative.dwx = wdot_B.x();
  derivative.dwy = wdot_B.y();
  derivative.dwz = wdot_B.z();

  return derivative;
}

State Simulator::AddScaledDerivative(
    const State& s,
    const StateDerivative& k,
    const double h) const
{
  // update state, while maintaining the header
  State out = s;

  out.pose.position.x  += h * k.dx;
  out.pose.position.y  += h * k.dy;
  out.pose.position.z  += h * k.dz;

  out.vel.linear.x += h * k.dvx;
  out.vel.linear.y += h * k.dvy;
  out.vel.linear.z += h * k.dvz;

  out.pose.orientation.x += h * k.dqx;
  out.pose.orientation.y += h * k.dqy;
  out.pose.orientation.z += h * k.dqz;
  out.pose.orientation.w += h * k.dqw;

  out.vel.angular.x += h * k.dwx;
  out.vel.angular.y += h * k.dwy;
  out.vel.angular.z += h * k.dwz;

  return out;
}

State Simulator::PropagateStateRK4(
    const State &prev,
    const Command &cmd,
    const double dt)
{
  // RK4
  const StateDerivative k1 = ComputeStateDerivative(prev, cmd);

  const State s2 = AddScaledDerivative(prev, k1, 0.5 * dt);
  const StateDerivative k2 = ComputeStateDerivative(s2, cmd);

  const State s3 = AddScaledDerivative(prev, k2, 0.5 * dt);
  const StateDerivative k3 = ComputeStateDerivative(s3, cmd);

  const State s4 = AddScaledDerivative(prev, k3, dt);
  const StateDerivative k4 = ComputeStateDerivative(s4, cmd);

  State next = prev;

  next.pose.position.x  += (dt/6.0) * (k1.dx  + 2.0*k2.dx  + 2.0*k3.dx  + k4.dx);
  next.pose.position.y  += (dt/6.0) * (k1.dy  + 2.0*k2.dy  + 2.0*k3.dy  + k4.dy);
  next.pose.position.z  += (dt/6.0) * (k1.dz  + 2.0*k2.dz  + 2.0*k3.dz  + k4.dz);

  next.vel.linear.x += (dt/6.0) * (k1.dvx + 2.0*k2.dvx + 2.0*k3.dvx + k4.dvx);
  next.vel.linear.y += (dt/6.0) * (k1.dvy + 2.0*k2.dvy + 2.0*k3.dvy + k4.dvy);
  next.vel.linear.z += (dt/6.0) * (k1.dvz + 2.0*k2.dvz + 2.0*k3.dvz + k4.dvz);

  next.pose.orientation.x += (dt/6.0) * (k1.dqx + 2.0*k2.dqx + 2.0*k3.dqx + k4.dqx);
  next.pose.orientation.y += (dt/6.0) * (k1.dqy + 2.0*k2.dqy + 2.0*k3.dqy + k4.dqy);
  next.pose.orientation.z += (dt/6.0) * (k1.dqz + 2.0*k2.dqz + 2.0*k3.dqz + k4.dqz);
  next.pose.orientation.w += (dt/6.0) * (k1.dqw + 2.0*k2.dqw + 2.0*k3.dqw + k4.dqw);

  next.vel.angular.x += (dt/6.0) * (k1.dwx + 2.0*k2.dwx + 2.0*k3.dwx + k4.dwx);
  next.vel.angular.y += (dt/6.0) * (k1.dwy + 2.0*k2.dwy + 2.0*k3.dwy + k4.dwy);
  next.vel.angular.z += (dt/6.0) * (k1.dwz + 2.0*k2.dwz + 2.0*k3.dwz + k4.dwz);

  // quaternion normalization
  {
    Eigen::Quaterniond q(next.pose.orientation.w, next.pose.orientation.x,
                         next.pose.orientation.y, next.pose.orientation.z);
    q.normalize();
    next.pose.orientation.w = q.w();
    next.pose.orientation.x = q.x();
    next.pose.orientation.y = q.y();
    next.pose.orientation.z = q.z();
  }

  // header + sim time
  next.id = prev.id;
  next.header.frame_id = prev.header.frame_id;

  rclcpp::Time sim_time_old = prev.header.stamp;
  rclcpp::Time sim_time_new = sim_time_prev_ + rclcpp::Duration::from_seconds(dt);
  next.header.stamp = sim_time_new;

  return next;
}
 
Simulator::Simulator()
: Node("simulator_node")
{
  RCLCPP_INFO(this->get_logger(), "Initialize Simulator node...");

  //QoS settings
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

  // Declare Parameters
  this->declare_parameter<double>("initial_time", 0.0);
  this->declare_parameter<double>("loop_rate_hz", 100.0);
  this->declare_parameter<std::string>("id", "ego");
  this->declare_parameter<std::string>("frame_id", "world");

  this->declare_parameter<double>("initial_x", 0.0);
  this->declare_parameter<double>("initial_y", 0.0);
  this->declare_parameter<double>("initial_z", 0.0);
  this->declare_parameter<double>("initial_vx", 0.0);
  this->declare_parameter<double>("initial_vy", 0.0);
  this->declare_parameter<double>("initial_vz", 0.0);

  this->declare_parameter<double>("initial_qx", 0.0);
  this->declare_parameter<double>("initial_qy", 0.0);
  this->declare_parameter<double>("initial_qz", 0.0);
  this->declare_parameter<double>("initial_qw", 1.0);
  this->declare_parameter<double>("initial_wx", 0.0);
  this->declare_parameter<double>("initial_wy", 0.0);
  this->declare_parameter<double>("initial_wz", 0.0);

  this->declare_parameter<double>("mass", 1.0);
  this->declare_parameter<double>("inertia_xx", 1.0);
  this->declare_parameter<double>("inertia_yy", 1.0);
  this->declare_parameter<double>("inertia_zz", 1.0);
  this->declare_parameter<double>("inertia_xy", 0.0);
  this->declare_parameter<double>("inertia_xz", 0.0);
  this->declare_parameter<double>("inertia_yz", 0.0);

  this->declare_parameter<double>("max_force", 1.0);
  this->declare_parameter<double>("max_torque", 1.0);

  GetParameters();

  RCLCPP_INFO(this->get_logger(),
    "Simulator Parameters: initial_time=%.3f, loop_rate_hz=%.3f, id=%s, frame_id=%s",
    initial_time_, loop_rate_hz_, o_initial_state_.id.c_str(), frame_id_.c_str());

  RCLCPP_INFO(this->get_logger(),
    "Initial Linear State: x=%.3f, y=%.3f, z=%.3f, vx=%.3f, vy=%.3f, vz=%.3f",
    o_initial_state_.pose.position.x, o_initial_state_.pose.position.y, o_initial_state_.pose.position.z,
    o_initial_state_.vel.linear.x, o_initial_state_.vel.linear.y, o_initial_state_.vel.linear.z);

  RCLCPP_INFO(this->get_logger(),
    "Initial Angular State: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f, wx=%.3f, wy=%.3f, wz=%.3f",
    o_initial_state_.pose.orientation.x, o_initial_state_.pose.orientation.y,
    o_initial_state_.pose.orientation.z, o_initial_state_.pose.orientation.w,
    o_initial_state_.vel.angular.x, o_initial_state_.vel.angular.y, o_initial_state_.vel.angular.z);

  RCLCPP_INFO(this->get_logger(),
    "Mass & Inertia: mass=%.3f, Ixx=%.3f, Iyy=%.3f, Izz=%.3f, Ixy=%.3f, Ixz=%.3f, Iyz=%.3f",
    mass_, inertia_(0,0), inertia_(1,1), inertia_(2,2),
    inertia_(0,1), inertia_(0,2), inertia_(1,2));
    
  RCLCPP_INFO(this->get_logger(),
    "Max Force & Torque: max_force=%.3f, max_torque=%.3f",
    max_force_, max_torque_);

  // Initialize last command (for safety)
  last_cmd_.id    = "";
  last_cmd_.force.x = 0.0;
  last_cmd_.force.y = 0.0;
  last_cmd_.force.z = 0.0;
  last_cmd_.torque.x = 0.0;
  last_cmd_.torque.y = 0.0;
  last_cmd_.torque.z = 0.0;

  // Subscribers Initialization
  sub_command_ = this->create_subscription<Command>(
  "command", qos_profile,
  std::bind(&Simulator::CallbackCommand, this, std::placeholders::_1));

  // Publishers Initialization
  pub_state_ = this->create_publisher<State>(
    "state", qos_profile);

  // Timer Initialization
  rclcpp::Time current_time = steady_clock.now();
  real_time_prev_ = current_time;
  sim_time_prev_ = rclcpp::Time(initial_time_);

  t_run_node_ = this->create_wall_timer(
      std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
      [this]() { this->Run(); }); 

  // Simulator Initialization
  o_initial_state_.header.frame_id = frame_id_;
  o_state_ = o_initial_state_;
  sim_time_prev_ = rclcpp::Time(initial_time_);
  real_time_prev_ = current_time;
}

Simulator::~Simulator()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Simulator node...");
}

void Simulator::GetParameters()
{
  // This function can be used to fetch parameters when needed
  this->get_parameter("initial_time", initial_time_);
  this->get_parameter("loop_rate_hz", loop_rate_hz_);
  this->get_parameter("id", o_initial_state_.id);
  this->get_parameter("frame_id", frame_id_);

  this->get_parameter("initial_x", o_initial_state_.pose.position.x);
  this->get_parameter("initial_y", o_initial_state_.pose.position.y);
  this->get_parameter("initial_z", o_initial_state_.pose.position.z);
  this->get_parameter("initial_vx", o_initial_state_.vel.linear.x);
  this->get_parameter("initial_vy", o_initial_state_.vel.linear.y);
  this->get_parameter("initial_vz", o_initial_state_.vel.linear.z);

  this->get_parameter("initial_qx", o_initial_state_.pose.orientation.x);
  this->get_parameter("initial_qy", o_initial_state_.pose.orientation.y);
  this->get_parameter("initial_qz", o_initial_state_.pose.orientation.z);
  this->get_parameter("initial_qw", o_initial_state_.pose.orientation.w);
  this->get_parameter("initial_wx", o_initial_state_.vel.angular.x);
  this->get_parameter("initial_wy", o_initial_state_.vel.angular.y);
  this->get_parameter("initial_wz", o_initial_state_.vel.angular.z);

  this->get_parameter("mass", mass_);

  double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
  this->get_parameter("inertia_xx", Ixx);
  this->get_parameter("inertia_yy", Iyy);
  this->get_parameter("inertia_zz", Izz);
  this->get_parameter("inertia_xy", Ixy);
  this->get_parameter("inertia_xz", Ixz);
  this->get_parameter("inertia_yz", Iyz);

  // fill in the MOI matrix
  inertia_ <<
    Ixx, Ixy, Ixz,
    Ixy, Iyy, Iyz,
    Ixz, Iyz, Izz;

  //compute inverse MOI matrix
  inertia_inv_ = inertia_.inverse();

  this->get_parameter("max_force", max_force_);
  this->get_parameter("max_torque", max_torque_);
}

void Simulator::Run()
{
  // time
  auto current_time = steady_clock.now();

  // time interval
  double time_dt = (current_time - real_time_prev_).seconds();
  if (time_dt <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
      "Non-positive time step detected: dt=%.6f. Skipping propagation.", time_dt);
    return;
  }
  else {
    sim_time_curr_ = sim_time_prev_ + rclcpp::Duration::from_seconds(time_dt);
  }
  real_time_prev_ = current_time;

  // propagate state
  Command cmd;
  {
    std::lock_guard<std::mutex> lock(mutex_command_);
    cmd = last_cmd_;
  }
  o_state_ = PropagateStateRK4(o_state_, cmd, time_dt);

  // publish state
  pub_state_->publish(o_state_);

  // update sim time
  sim_time_prev_ = sim_time_curr_;
}

int main(int argc, char ** argv)
{  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());

  rclcpp::shutdown();
  return 0;
}