# each motor has a unique id and specifies its target length, current position, and commanded speed.  Speed is positive if the motor is extending and negative if it is contracting.  Done is true if the motor has reached the target within the tolerance.

uint8 min_length
uint8 RANGE
uint8 MAX_RANGE
uint8 MIN_RANGE
uint8 RANGE024
uint8 RANGE135
int8 max_speed
float64 tol
float64 low_tol
float64 P
float64 I
float64 D
float64 dist_weight
float64 ang_weight
float64 prog_weight