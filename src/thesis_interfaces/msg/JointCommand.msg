# Timestamp associated with the command.
builtin_interfaces/Time stamp

# Unique command identifier for traceability and logging.
string command_id

# Ordered list of manipulator joint names.
string[] joint_names

# Desired joint positions in radians.
float64[] positions

# Requested motion duration.
builtin_interfaces/Duration duration
