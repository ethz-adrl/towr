/**
@file    Main.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include "../../include/xpp/ros/marker_array_builder.h"


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "nlp_optimizer_node");

	xpp::ros::MarkerArrayBuilder visualizer("nlp_zmp_publisher");
	xpp::ros::NlpOptimizerNode xpp_optimizer_node(visualizer);

	ros::spin();

	return 1;
}

