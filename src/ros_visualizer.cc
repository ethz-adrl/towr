/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include <xpp/ros/ros_visualizer.h>
#include <xpp/ros/ros_helpers.h>

#include <xpp/ros/topic_names.h>

namespace xpp {
namespace ros {

using VectorXd = Eigen::VectorXd;
using MarkerMsg   = visualization_msgs::Marker;
using MarkerArrayMsg = visualization_msgs::MarkerArray;
using EndeffectorID = xpp::utils::EndeffectorID;

RosVisualizer::RosVisualizer ()
{
  ::ros::NodeHandle n;

  sub_  = n.subscribe(xpp_msgs::robot_trajectory_cart, 1,
                      &RosVisualizer::TrajectoryCallback, this);

  contacts_sub_ = n.subscribe(xpp_msgs::contact_vector, 1,
                              &RosVisualizer::ContactsCallback, this);


  ros_publisher_optimized_ = n.advertise<MarkerArrayMsg>(xpp_msgs::rviz_optimized, 1);
  ros_publisher_optimized_single_ = n.advertise<MarkerMsg>(xpp_msgs::rviz_optimized_single, 1);
}

RosVisualizer::~RosVisualizer ()
{
}

void
RosVisualizer::TrajectoryCallback (const TrajMsg::ConstPtr& traj_msg)
{
  msg_builder_.robot_traj_ = RosHelpers::RosToXppCart(*traj_msg);
  ROS_INFO_STREAM("Received new robot trajectory");


  MarkerArrayMsg msg;
  msg_builder_.AddBodyTrajectory(msg);
  msg_builder_.AddZmpTrajectory(msg);
  msg_builder_.AddSupportPolygons(msg);
  msg_builder_.AddStartStance(msg);
  msg_builder_.AddFootholds(msg);


  auto first_state = RosHelpers::RosToXpp(traj_msg->states.front());
  Eigen::Vector2d start = first_state.GetBase().lin.Get2D().p;
  msg_builder_.AddPoint(msg, start, "start", visualization_msgs::Marker::CYLINDER);

  ros_publisher_optimized_.publish(msg);
}

void
RosVisualizer::ContactsCallback (const ContactVecMsg& contact_msg)
{
  VecContacts contacts = RosHelpers::RosToXpp(contact_msg);
  MarkerArrayMsg msg;

  // already publishing using trajectory callback
//  msg_builder_.AddFootholds(msg, contacts, "footholds", visualization_msgs::Marker::SPHERE, 1.0);
//  ros_publisher_optimized_.publish(msg);
}

void
RosVisualizer::Visualize () const
{
  // spring_clean_ remove all this
  const auto com_motion = GetComMotion();
  auto structure        = GetMotionStructure();
  auto contacts         = GetContacts();
  auto leg_ids          = structure.GetContactIds();
  auto start_stance     = structure.GetStartStance();
  double com_height     = com_motion->GetZHeight();

  VecContacts footholds;
  for (int i=0; i<contacts.size(); ++i) {
    xpp::opt::Contact f;
    f.ee    = leg_ids.at(i);
    f.p.x() = contacts.at(i).x();
    f.p.y() = contacts.at(i).y();
    f.p.z() = 0.0;
    footholds.push_back(f);
  }

//  std::cout << leg_ids.size() << std::endl;
//  for (auto ee : leg_ids) {
//    std::cout << "ee: " << ee << std::endl;
//  }
//
//  std::cout << contacts.size() << std::endl;
//  for (auto contact : contacts) {
//    std::cout << "contact: " << contact << std::endl;
//  }


  MarkerArrayMsg msg;
//  msg_builder_.AddPoint(msg, com_motion->GetCom(0.0).p - motion_params_->offset_geom_to_com_.segment<2>(0),
//                        "start", visualization_msgs::Marker::CYLINDER);
//  msg_builder_.AddStartStance(msg, start_stance);
//  msg_builder_.AddFootholds(msg, footholds, "footholds", visualization_msgs::Marker::SPHERE, 1.0);
//  msg_builder_.AddSupportPolygons(msg, structure, contacts);
//  msg_builder_.AddBodyTrajectory(msg, *com_motion, motion_params_->offset_geom_to_com_, structure, "body", 1.0);
//  msg_builder_.AddZmpTrajectory(msg, *com_motion, structure, com_height, "zmp_4ls");


//  msg_builder_.AddPendulum(msg, *com_motion, structure, com_height, "pendulum", 1.0);
//
//
//  ROS_INFO("Trajectory received, forwarding to rviz...");
//  double playback_speed = 0.5;
//  double dt = 0.01; // same as discretization of pendulum states
//  ::ros::Rate loop_rate(1.0/dt*playback_speed);
//
//  for (int i=0; i<msg.markers.size(); ++i)
//  {
//    ros_publisher_optimized_single_.publish(msg.markers.at(i));
//    loop_rate.sleep();
//  }


//  double gap_center_x = 0.45;
//  double gap_width_x = 0.2;
//  double ellipse_width = 1.0;
//  msg_builder_.AddLineStrip(msg, gap_center_x, gap_width_x, "gap");
//  msg_builder_.AddEllipse(msg, gap_center_x, 0.0, gap_width_x, ellipse_width, "ellipse");

//  static int n_markers_first_iteration = msg.markers.size();
//  for (int i=n_markers_first_iteration; i<msg.markers.size(); ++i) {
//    msg.markers.at(i).type = visualization_msgs::Marker::DELETE;
//    msg.markers.at(i).color.a = 0.0;
//  }

//  ros_publisher_optimized_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
