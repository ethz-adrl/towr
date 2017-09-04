/**
@file    Main.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */

#include <ros/init.h>

#include <xpp/ros/nlp_user_input_node.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nlp_user_input_node");

  xpp::ros::NlpUserInputNode nlp_user_input_node;

  ros::spin();

  return 1;
}

