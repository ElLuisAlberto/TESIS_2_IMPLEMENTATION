# Reference accepted by the simulation adapter for one arm movement.
# This is an execution reference, not a safety decision.
builtin_interfaces/Time stamp
builtin_interfaces/Time start_time
string command_id

# PENDING, ACCEPTED, SUCCEEDED, FAILED, CANCELED, REJECTED or DRY_RUN.
string status
string detail

string[] joint_names
float64[] start_positions
float64[] target_positions
float64 duration_sec
