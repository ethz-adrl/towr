# The command for the robot specified by the user

xpp_msgs/StateLin3d    goal_lin           # the linear state to reach (x,y,z and derivatives)
xpp_msgs/StateLin3d    goal_ang           # the angular state to reach (roll, pitch, yaw and derivatives)
int32                  motion_type        # Type of Motion (Walk, Trott, Bound, Camel)
bool                   replay_trajectory  # Replay the already optimized trajectory in RVIZ
bool                   use_solver_snopt   # otherwise Ipopt.
bool                   optimize           # reoptimize for changed parameters
bool                   publish_traj       # publish optimization results (e.g. for controller)

# some information about the used terrain (e.g stairs, gap, slope)
int32                  terrain_id     
int32                  gait_id   
float64                total_duration          
