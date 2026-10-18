# Lifecycle and measured progress of one Cartesian trajectory intention.
builtin_interfaces/Time stamp
string command_id

# RUNNING, PAUSED, SUCCEEDED, FAILED, CANCELED or REJECTED.
string status
string detail

float64 progress
float64 speed_scale
float64 planned_duration_sec
float64 remaining_distance_m
float64 position_error_m
float64 orientation_error_rad

# Absolute final tool pose in the command frame.
string frame_id
float64 target_x
float64 target_y
float64 target_z
float64 target_roll
float64 target_pitch
float64 target_yaw
