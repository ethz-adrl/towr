/**
@file    Main.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */

#include <ros/ros.h>
#include <xpp/ros/xpp_optimizer_node.h>


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "xpp_optimizer_node");

	xpp::ros::XppOptimizerNode xpp_optimizer_node;

	ros::spin();

	return 1;
}

