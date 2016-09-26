# Optimizes the trajectory and footholds to reach a given goal state
# The optimized values are saved in the server and can be retrieved by another
# service call.

PhaseInfo[] phases    # not optimized, but puts the motion into context
Spline[] splines
xpp_msgs/Foothold[] footholds
