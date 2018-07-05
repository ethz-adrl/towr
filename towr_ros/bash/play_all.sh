#!/bin/bash
#title      :Plays back all rosbag files in a specific folder
#author     :Alexander W. Winkler


# change to directory where the script is stored so relative paths are correct
my_dir="$(dirname "$0")"
cd $my_dir

speed=1.0

video_folder=$my_dir/video1/*
for bag_file in $video_folder
do	
	# draw the terrain
	rosbag play $bag_file \
	--quiet \
	--topics xpp/user_command_saved \
	xpp/params xpp/user_command_saved:=/xpp/user_command \
	-d 0.3 `#wait 0.01s after opening bag` \
	
	# publish the paramters and terrain
	rosbag play $bag_file \
	--topics xpp/state \
	-r $speed
done

