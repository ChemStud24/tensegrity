# This is a message that represents the trajectory following and MPC predictions for a tensegrity robot.  Trajectory is a sequence of points along the desired trajectory.  COMs is a sequence of predicted centers of mass for the robot.  PAs is a sequence of predicted principal axis unit vectors.  This message depends on geometry_msgs/Point

geometry_msgs/Point[] trajectory
geometry_msgs/Point[] COMs
geometry_msgs/Point[] PAs
int8 trajectory_segment