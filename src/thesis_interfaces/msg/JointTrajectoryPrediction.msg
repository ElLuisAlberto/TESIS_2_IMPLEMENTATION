# Discrete joint prediction anchored at one measured state.
builtin_interfaces/Time stamp
builtin_interfaces/Time origin_stamp
string command_id

# intent, jog, execution or zero.
string source
string[] joint_names

uint32 sample_count
float64 sample_period
float64 horizon
float64 effective_duration

# sample_times[k] corresponds to positions[k * joint_count:(k + 1) * joint_count].
float64[] sample_times
float64[] positions
