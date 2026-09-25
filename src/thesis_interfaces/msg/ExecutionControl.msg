# Runtime safety decision for an already accepted trajectory.
builtin_interfaces/Time stamp
string command_id

# ALLOW, WARNING, REDUCTION or STOP.
string state

# Requested fraction of the nominal speed. Zero means stop.
float64 speed_scale

# Geometric evidence used by the supervisor and visualizer.
float64 minimum_clearance
string limiting_segment

# All absent events use -1.0. Nominal TTC is never overwritten by scaling.
float64 protective_margin
float64 time_to_protective_volume
float64 time_to_collision
float64 supervised_time_to_protective_volume
float64 supervised_time_to_collision

# Location of the minimum clearance inside the sampled short horizon.
# minimum_time_from_now is relative to the latest measured configuration.
float64 minimum_time_from_now
uint32 minimum_sample_index
uint32 minimum_sample_count
float64 minimum_horizon_fraction

# Machine-readable reason and human-readable explanation.
string reason_code
string reason

# Raw request and temporal-filter diagnostics.  state and speed_scale above
# always represent the filtered decision that may be applied to the robot.
string requested_state
float64 requested_speed_scale
string requested_reason_code
uint32 recovery_count
uint32 recovery_required_count
bool transition
string transition_reason


# Complete geometric evidence for the selected supervised horizon.
float64 current_clearance
float64 nominal_clearance
float64 supervised_clearance
geometry_msgs/Point closest_robot_point
geometry_msgs/Point obstacle_center_at_minimum
geometry_msgs/Point capsule_start
geometry_msgs/Point capsule_end
float64 capsule_radius
uint32 evaluated_combinations
