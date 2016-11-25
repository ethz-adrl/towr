/**
@file    Main.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */

#include <xpp/ros/nlp_user_input_node.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nlp_user_input_node");

  xpp::ros::NlpUserInputNode nlp_user_input_node;
  ros::Rate loop_rate(nlp_user_input_node.kLoopRate_);

  static double t_left_min = 0.2; // to avoid jumps
  while (ros::ok())
  {
    nlp_user_input_node.PublishCommand();

    double t_left = nlp_user_input_node.t_left_ - 1./nlp_user_input_node.kLoopRate_;
    nlp_user_input_node.t_left_ = t_left > t_left_min ? t_left : t_left_min; // mpc make this different from zero?

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 1;
}

