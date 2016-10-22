#!/bin/bash

# terminate script on first error
set -e

# assumes the ros workspaces setup.bash is sources here.
source ~/.bashrc

# this is the location where a rosnode is started from. Must be where the 
# Ipopt config files is stored.
roscd xpp_opt/config
export ROS_HOME=`pwd`
export ROS_LOG_DIR=~/.ros/log

roslaunch xpp_ros xpp.launch

unset ROS_HOME
