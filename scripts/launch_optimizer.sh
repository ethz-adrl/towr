#!/bin/bash
#title           :X++ Dynamic Locomotion Framework execution script
#description     :This script prepares the environment to launch ros nodes.
#author		     :Alexander W. Winkler
#date            :20161025
#usage		     :This script sources the catkin workspace, navigates to the 
#                 directory where the ipopt config file is stored and launches 
#                 all the neccessary nodes to run the optimization for hyq.
#
#                 true     starts the visualizer and rviz as well
#==============================================================================

main() 
{
 source_ros_workspace
 launch_optimizer_from_config_dir $1
}

function source_ros_workspace() 
{
 set -e # terminate script on first error

 my_dir="$(dirname "$0")"
 cd $my_dir
 source ./../../../../devel/setup.bash
}


function launch_optimizer_from_config_dir()
{
  # this is the location where a rosnode is started from. Must be where the 
  # Ipopt config files is stored.
 roscd xpp_opt/config
 export ROS_HOME=`pwd`
 export ROS_LOG_DIR=~/.ros/log

 # launch optimizer with or without visualization (default off)
 roslaunch xpp_ros xpp.launch vis:=$1
}


main "$@"
