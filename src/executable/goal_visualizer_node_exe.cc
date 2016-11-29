/**
@file    goal_visualizer_node_exe.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2016
@brief   Visualizes a goal state in rviz that is subscribes to.
 */

#include <xpp/ros/marker_array_builder.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/UserCommand.h>   // listen to goal state
#include <ros/ros.h>

static ros::Publisher rviz_pub;
static ros::Subscriber goal_sub;

void CallbackGoal(const xpp_msgs::UserCommand& msg)
{
  auto goal = xpp::ros::RosHelpers::RosToXpp(msg.goal);

  visualization_msgs::MarkerArray msg_rviz;
  xpp::ros::MarkerArrayBuilder msg_builder_;
  msg_builder_.AddPoint(msg_rviz, goal.Get2D().p, "goal", visualization_msgs::Marker::CUBE);
  rviz_pub.publish(msg_rviz);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "goal_visualizer");
	ros::NodeHandle n;

	rviz_pub = n.advertise<visualization_msgs::MarkerArray>(xpp_msgs::rviz_fixed, 1);
	goal_sub = n.subscribe(xpp_msgs::goal_state_topic, 1, CallbackGoal);

	ros::spin();
	return 1;
}
