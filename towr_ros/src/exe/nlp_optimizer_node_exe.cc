/**
@file    nlp_optimizer_node.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */

#include <ros/ros.h>
#include <towr_ros/nlp_optimizer_node.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nlp_optimizer_node");
  towr::NlpOptimizerNode xpp_optimizer_node;
  ros::spin();

  return 1;
}

