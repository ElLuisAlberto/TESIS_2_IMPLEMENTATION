# Timestamp and reference frame used for the geometric evaluation.
builtin_interfaces/Time stamp
string reference_frame

# ALLOW, WARNING, REDUCTION or STOP.
string state

# Surface-to-surface clearance in metres. Negative means overlap.
float64 minimum_clearance
string limiting_segment

# Geometric witnesses used for diagnostics and RViz rendering.
geometry_msgs/Point closest_robot_point
geometry_msgs/Point obstacle_center
float64 obstacle_radius
