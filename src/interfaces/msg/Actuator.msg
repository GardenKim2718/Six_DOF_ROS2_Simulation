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

# Thruster command (12 thruster)
float64[12] thruster_cmd  # thruster_cmd[0-11]  [N]

# RWA command (4 RWA case)
float64[4] rwa_torque_cmd  # rwa_torque_cmd[0-3]  [N*m]