# This is a message that represents the MPC-selected actions for a tensegrity robot.  The variable actions is a sequence of k strings that represent the depth-k motion plan from MPC.  COMs and PAs are the current center of mass and principal axis plus the k predicted future COMs and PAs corresponding to the selected sequence of actions. This message depends on geometry_msgs/Point.

string[] actions
float64 cost
geometry_msgs/Point[] COMs
geometry_msgs/Point[] PAs
geometry_msgs/Point[] endcaps
float64 dist_weight
float64 ang_weight
float64 prog_weight