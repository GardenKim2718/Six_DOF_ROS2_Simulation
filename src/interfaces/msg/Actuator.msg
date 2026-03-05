# Actuator.msg
# ------------------------------------------------------------------
# Output force and torque command from spacecraft's actuators
#
# Coordinate conventions (recommended):
# - force  : body frame [N]
# - torque : body frame [N*m]
#
# Use header.frame_id if you want to indicate explicitly which frame
# the command is defined in (e.g., "base_link", "body", or "world").
# ------------------------------------------------------------------

# Optional identifier for multi-agent systems
string id    # robot/agent name (e.g., "robot1")

# 3D force command, typically expressed in body frame
geometry_msgs/Vector3 force   # [N], (fx, fy, fz)

# 3D torque command, typically expressed in body frame
geometry_msgs/Vector3 torque  # [N*m], (tx, ty, tz)