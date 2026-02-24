# This is a timestamped message that represents a state consisting of 3 poses, encoder counts, and motor speeds
# Each pose represents a 7D pose (position + quaternion orientation)
# Encoder counts and motor speeds are arrays of values

Header header
geometry_msgs/Pose[] poses
float64[] encoder_lengths
float64[] motor_speeds
