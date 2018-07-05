# The command for the robot specified by the user

xpp_msgs/StateLin3d    goal_lin           # the linear state to reach (x,y,z and derivatives)
xpp_msgs/StateLin3d    goal_ang           # the angular state to reach (roll, pitch, yaw and derivatives)
float64                total_duration     # the total time to reach the goal state
bool                   replay_trajectory  # Replay the already optimized trajectory in RVIZ
bool                   optimize           # run TOWR optimization
int32                  terrain_id         # some information about the used terrain (e.g stairs, gap, slope)
int32                  gait_id            # Type of Motion (Walk, Trott, Bound, Camel)
