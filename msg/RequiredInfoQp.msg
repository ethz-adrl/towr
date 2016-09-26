# Optimizes the trajectory and footholds to reach a given goal state
# The optimized values are saved in the server and can be retrieved by another
# service call.

xpp_msgs/StateLin3d curr_state
xpp_msgs/Foothold[] curr_stance 
xpp_msgs/Foothold[] steps
bool start_with_com_shift     #whether to first shift cog before lifting leg