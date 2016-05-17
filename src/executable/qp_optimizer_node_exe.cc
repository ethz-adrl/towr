/**
@file    Main.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */


#include <xpp/ros/qp_optimizer_node.h>


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "qp_optimizer_node");

	xpp::ros::QpOptimizerNode qp_optimizer_node;

	ros::Rate loop_rate(1000); // don't know if this acutally makes it faster
	ros::spin();

	return 1;
}

