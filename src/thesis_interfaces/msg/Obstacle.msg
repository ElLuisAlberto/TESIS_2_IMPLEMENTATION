# Obstacle estimate from any perception or simulation source.
# The center and velocity are expressed in header.frame_id; timestamp is acquisition time.
std_msgs/Header header
geometry_msgs/Point center
geometry_msgs/Vector3 velocity
float64 radius
float64 uncertainty
