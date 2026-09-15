# Final supervisor decision for one candidate command.
builtin_interfaces/Time stamp
string command_id

# True only when the supervisor authorizes forwarding.
# Does not confirm execution by the robot or simulator.
bool accepted

# ALLOW, WARNING, REDUCTION or STOP.
string state

# Machine-readable reason and human-readable explanation.
string reason_code
string reason
