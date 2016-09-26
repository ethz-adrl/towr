# Only the current state of the system being optimized. Goal and additional 
# information on HOW to solve the problem come from the user.

# Optimizes the trajectory and footholds to reach a given goal state
# The optimized values are saved in the server and can be retrieved by another
# service call.

xpp_msgs/StateLin3d curr_state
xpp_msgs/Foothold[] curr_stance
int32 curr_swingleg           # can also be no swingleg, if in stance phase