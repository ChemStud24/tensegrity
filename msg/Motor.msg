# each motor has a unique id and specifies its target length, current position, and commanded speed.  Speed is positive if the motor is extending and negative if it is contracting.  Done is true if the motor has reached the target within the tolerance.  The three error terms (proportional, derivative, and cumulative) are those used in the PID calculation for this motor.

int8 id
float64 position
float64 target
float64 speed
bool done
float64 error
float64 d_error
float64 cum_error