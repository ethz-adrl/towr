#!/bin/bash
#title           :Remaps xpp/state_des in a bag file to another name
#input           :the name of the bag file
#author		 :Alexander W. Winkler

new_topic_name=xpp/state_curr
bag_file=$1

rosrun rosbag topic_renamer.py xpp/state_des ${bag_file}.bag $new_topic_name ${bag_file}_remap.bag

                           
