#!/bin/bash
#title         :Plays back rosbag files of optimization iterations
#author	       :Alexander W. Winkler


# change to directory where the script is stored to relative paths are correct
my_dir="$(dirname "$0")"
cd $my_dir

speed=2.0

# publish the optimization parameters, but wait 0.5s to make sure
# subscribers are connected
rosbag play towr_trajectory.bag --topics xpp/params -d 0.5


# use rosbag API to remap the iteration topic to current robot state
rosbag play towr_trajectory.bag \
	--quiet \
	--topics iter1 \
	iter1:=/xpp/state \
	-d 0.01 `#wait 0.01s after opening bag` \
	-r $speed \
&& \
rosbag play towr_trajectory.bag \
	--topics iter10 \
	-d 0.01 \
	iter10:=/xpp/state \
	-r $speed \
&& \
rosbag play towr_trajectory.bag \
	--topics iter20 \
	-d 0.01 \
	iter20:=/xpp/state \
	-r $speed \
                           
