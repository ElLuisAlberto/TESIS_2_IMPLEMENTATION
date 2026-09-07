# Prediction produced synchronously for one candidate command.
builtin_interfaces/Time stamp
string command_id
string reference_frame

# ALLOW, WARNING, REDUCTION or STOP based on projected clearance.
string state
float64 minimum_clearance
string limiting_segment

# Location of the worst sampled configuration along the trajectory.
uint32 sample_index
uint32 sample_count
float64 trajectory_fraction

# Limiting capsule at the worst sampled configuration.
geometry_msgs/Point capsule_start
geometry_msgs/Point capsule_end
float64 capsule_radius
