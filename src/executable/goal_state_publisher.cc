/**
@file    Main.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */


#include <ros/ros.h>
#include <xpp_opt/StateLin3d.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "goal_state_publisher");
  ros::NodeHandle n;
  ros::Publisher goal_state_pub = n.advertise<xpp_opt::StateLin3d>("goal_state", 10);

  xpp_opt::StateLin3d msg;
  if (argc!=3)
    ROS_FATAL("Please specify goal xy-positions as parameter");
  msg.pos.x = atof(argv[1]);
  msg.pos.y = atof(argv[2]);

  ros::Rate poll_rate(100);
  while(goal_state_pub.getNumSubscribers() == 0) {
    poll_rate.sleep();
  }

  goal_state_pub.publish(msg);
  ros::spin();

  return 1;
}

