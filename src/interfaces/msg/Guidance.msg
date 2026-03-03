# Guidance.msg
# ------------------------------------------------------------------
# 3D reference Guidance target state for control
#
# Coordinate conventions (recommended):
# - pose.position/orientation : world frame
# - vel.linear/angular        : body frame
# - accel.linear/angular      : body frame
# ------------------------------------------------------------------

# Optional identifier for multi-agent systems
string id    # robot/agent name (e.g., "robot1")

# Timestamp + coordinate frame of the pose
std_msgs/Header header   # header.stamp : simulation time
                         # header.frame_id : world frame for pose

# APDG Guidance Flag
bool linear_guidance_active   # true if linear guidance is active
bool angular_guidance_active  # true if angular guidance is active

# 3D acceleration = linear (m/s^2) + angular (rad/s^2)
geometry_msgs/Accel accel # accel.linear.x/y/z   [m/s^2] (Inertial frame)
                          # accel.angular.x/y/z  [rad/s^2] (Body frame)