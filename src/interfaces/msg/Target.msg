# Target.msg
# ------------------------------------------------------------------
# 3D Target state for guidance
#
# Coordinate conventions (recommended):
# - pose.position/orientation : world frame
# ------------------------------------------------------------------

# Optional identifier for multi-agent systems
string id                # robot/agent name (e.g., "robot1")

std_msgs/Header header   # header.stamp : simulation time
                         # header.frame_id : world frame for pose

# 3D pose = position (m) + orientation quaternion (unit quaternion)
geometry_msgs/Pose pose  # pose.position.x/y/z  [m]
                         # pose.orientation.x/y/z/w  (unit quaternion)