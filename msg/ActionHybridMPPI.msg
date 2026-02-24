# This is a timestamped message containing a control_type (astar or mppi), 
# mppi_actions (2D float matrix of control signals), control_interval (length of time per mppi_action), 
# primitive_actions (string array of primitives)

Header header
string control_type
float64[] mppi_actions
float64 control_interval
string[] primitive_actions