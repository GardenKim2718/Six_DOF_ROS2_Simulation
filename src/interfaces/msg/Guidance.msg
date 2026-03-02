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

# 3D pose = position (m) + orientation quaternion (unit quaternion)
geometry_msgs/Pose pose  # pose.position.x/y/z  [m]
                         # pose.orientation.x/y/z/w  (unit quaternion)

# 3D velocity = linear (m/s) + angular (rad/s)
geometry_msgs/Twist vel  # vel.linear.x/y/z   [m/s]
                         # vel.angular.x/y/z  [rad/s]

# 3D acceleration = linear (m/s^2) + angular (rad/s^2)
geometry_msgs/Accel accel # accel.linear.x/y/z   [m/s^2]
                          # accel.angular.x/y/z  [rad/s^2]