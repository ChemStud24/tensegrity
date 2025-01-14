# This is a message that represents state of a tensegrity robot to be passed to a model-predictive controller.  Trajectory is a sequence of waypoints along the desired trajectory.  prev_action is the previous action that MPC will use as a key to its internal lookup table.  reverse_the_gait is a boolean that tells the planner if the robot is rolling forward or backward during this segment of the trajectory.  This message depends on geometry_msgs/Point.

geometry_msgs/Point[] trajectory
string prev_action
bool reverse_the_gait
bool bar_height_changed