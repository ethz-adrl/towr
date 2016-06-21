# Optimizes the trajectory and footholds to reach a given goal state
# The optimized values are saved in the server and can be retrieved by another
# service call.

StateLin3d curr_state
Foothold[] curr_stance
int32 curr_swingleg 
bool start_with_com_shift     #whether to first shift cog before lifting leg