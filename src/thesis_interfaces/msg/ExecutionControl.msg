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
float64 time_to_collision

# Location of the minimum clearance inside the sampled short horizon.
# minimum_time_from_now is relative to the latest measured configuration.
float64 minimum_time_from_now
uint32 minimum_sample_index
uint32 minimum_sample_count
float64 minimum_horizon_fraction

# Machine-readable reason and human-readable explanation.
string reason_code
string reason
