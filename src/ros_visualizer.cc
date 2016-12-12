/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include <xpp/ros/ros_visualizer.h>
#include <xpp/opt/phase.h>

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/marker_array_builder.h>
#include <xpp/ros/topic_names.h>

namespace xpp {
namespace ros {

using VectorXd = Eigen::VectorXd;

RosVisualizer::RosVisualizer ()
{
  ::ros::NodeHandle n;
  ros_publisher_optimized_ = n.advertise<visualization_msgs::MarkerArray>(xpp_msgs::rviz_optimized, 1);
  ros_publisher_fixed_     = n.advertise<visualization_msgs::MarkerArray>(xpp_msgs::rviz_fixed, 1);
}

RosVisualizer::~RosVisualizer ()
{
}

void
RosVisualizer::VisualizeCurrentState (const State& curr,
                                      const VecFoothold& start_stance) const
{
  visualization_msgs::MarkerArray msg;
  MarkerArrayBuilder msg_builder;

  msg_builder.AddPoint(msg, curr.p, "current", visualization_msgs::Marker::CYLINDER);


  // zmp_ see if i can eliminate these somehow
  std::vector<xpp::opt::ContactDerived> contacts_initial;
  for (auto f : start_stance) {
    xpp::opt::ContactDerived c;
    c.ee = static_cast<xpp::opt::EndeffectorID>(f.leg);
    c.id = f.id;
    c.p = f.p;

    contacts_initial.push_back(c);
  }


  msg_builder.AddStartStance(msg, contacts_initial);

  ros_publisher_fixed_.publish(msg);
}

void
RosVisualizer::Visualize () const
{
  const auto com_motion = GetComMotion();
  auto structure        = GetMotionStructure();
  auto contacts         = GetContacts();
  auto leg_ids          = structure.GetContactIds();
  auto start_stance     = structure.GetStartStance();

  std::vector<xpp::opt::ContactDerived> footholds;
  for (int i=0; i<contacts.size(); ++i) {
    xpp::opt::ContactDerived f;
    f.ee    = static_cast<xpp::opt::EndeffectorID>(leg_ids.at(i));
    f.p.x() = contacts.at(i).x();
    f.p.y() = contacts.at(i).y();
    f.p.z() = 0.0;
    footholds.push_back(f);
  }





  visualization_msgs::MarkerArray msg;
  MarkerArrayBuilder msg_builder_;
  msg_builder_.AddStartStance(msg, start_stance);
  msg_builder_.AddFootholds(msg, footholds, "footholds", visualization_msgs::Marker::CUBE, 1.0);
  msg_builder_.AddSupportPolygons(msg, structure, contacts);
  msg_builder_.AddCogTrajectory(msg, *com_motion, structure, "cog", 1.0);
  msg_builder_.AddZmpTrajectory(msg, *com_motion, structure, 0.58, "zmp_4ls", 0.2);



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

  ros_publisher_optimized_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
