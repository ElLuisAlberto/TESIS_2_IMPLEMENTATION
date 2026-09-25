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


# Complete geometric evidence for the global future minimum.
float64 minimum_time_from_now
float64 current_clearance
float64 nominal_clearance
float64 supervised_clearance
geometry_msgs/Point closest_robot_point
geometry_msgs/Point obstacle_center_at_minimum
uint32 evaluated_combinations


# Package 5 uncertainty and TTC evidence. Absent events use -1.0.
float64 protective_margin
float64 time_to_protective_volume
float64 time_to_collision
float64 supervised_time_to_protective_volume
float64 supervised_time_to_collision
