# State.msg
# ------------------------------------------------------------------
# Evaluation message for recording and analysis of GNC performance.
#
# Coordinate conventions (recommended):
# - pose.position/orientation : world frame
# - vel.linear/angular        : body frame
# - accel.linear/angular      : body frame
#
# Modify the frame_id in the header to match your system:
#   - "world"
#   - "map"
#   - "odom"
#   - "base_link"
# ------------------------------------------------------------------
string id    # robot/agent name (e.g., "robot1")

# Timestamp + coordinate frame of the pose
std_msgs/Header header   # header.stamp : simulation time
                         # header.frame_id : world frame for pose

# Force command vs Actual Force
geometry_msgs/Vector3 force_command  # [N]
geometry_msgs/Vector3 force_actual   # [N]

# Torque Command vs Actual Torque
geometry_msgs/Vector3 torque_command  # [N*m]
geometry_msgs/Vector3 torque_actual   # [N*m]
geometry_msgs/Vector3 rwa_torque      # [N*m]
geometry_msgs/Vector3 thruster_torque # [N*m]

# Guidnace vs Acceleration & Angular Acceleration
geometry_msgs/Accel guidance_accel  # accel.linear.x/y/z   [m/s^2] (Inertial frame)
                                    # accel.angular.x/y/z  [rad/s^2] (Body frame)
geometry_msgs/Accel actual_accel    # accel.linear.x/y/z   [m/s^2] (Inertial frame)
                                    # accel.angular.x/y/z  [rad/s^2] (Body frame)
