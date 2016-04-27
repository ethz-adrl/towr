# Optimizes the trajectory and footholds to reach a given goal state
# The optimized values are saved in the server and can be retrieved by another
# service call.

StateLin3d curr_state
Foothold[4] curr_stance  #for now always start optimization with 4 legs on ground