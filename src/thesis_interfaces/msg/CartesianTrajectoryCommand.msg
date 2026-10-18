# Robot-independent Cartesian trajectory intention.
builtin_interfaces/Time stamp
string command_id
string frame_id

# RELATIVE interprets the six values as deltas from the measured tool pose.
# ABSOLUTE interprets them as a target pose in frame_id.
string mode

# Metres and intrinsic roll/pitch/yaw radians.
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw

# Nominal operator request. The preventive supervisor may scale it to zero.
float64 linear_speed_mps
float64 angular_speed_radps
