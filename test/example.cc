/*!
 * \file   example.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 4, 2014
 * \brief  An example implementation of how to generate a trajectory.
 */


#include <xpp_opt/FootholdSequence.h>
#include <xpp/zmp/zmp_optimizer.h>
#include <xpp_opt/FootholdSequence.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <iostream> //std::cout, std::fixed
#include <iomanip>  //std::setprecision


std::string frame_id = "world";
visualization_msgs::MarkerArray footsteps_msg_;



void AddTrajectory(visualization_msgs::MarkerArray& msg,
                   xpp::zmp::SplineContainer zmp_splines,
                   std_msgs::ColorRGBA color,
                   std::string frame_id)
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (double t(0.0); t < zmp_splines.T; t+= 0.03)
  {
    xpp::utils::Point2d cog_state;
    zmp_splines.GetCOGxy(t, cog_state);


    visualization_msgs::Marker marker;
    marker.id = i++;
    marker.pose.position.x = cog_state.p.x();
    marker.pose.position.y = cog_state.p.y();
    marker.pose.position.z = 0.0;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
//    marker.lifetime = ros::Duration(10);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
    marker.color = color;

    msg.markers.push_back(marker);
  }
}




void AddFootholds(
    visualization_msgs::MarkerArray& msg,
    const std::vector<xpp::hyq::Foothold>& H_footholds,
    std::string frame_id,
    int32_t type = visualization_msgs::Marker::SPHERE,
    double alpha = 1.0)
{

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (int j=0; j<H_footholds.size(); ++j) {

    ::geometry_msgs::Point f_curr;
    f_curr.x = H_footholds.at(j).p.x();
    f_curr.y = H_footholds.at(j).p.y();
    f_curr.z = H_footholds.at(j).p.z();

//    ::geometry_msgs::Point f_next1;
//    f_next1.x = H_footholds.at(j+1).p.x();
//    f_next1.y = H_footholds.at(j+1).p.y();
//    f_next1.z = H_footholds.at(j+1).p.z();
//
//    ::geometry_msgs::Point f_next2;
//    f_next2.x = H_footholds.at(j+1).p.x();
//    f_next2.y = H_footholds.at(j+1).p.y();
//    f_next2.z = H_footholds.at(j+1).p.z();
//
//    ::geometry_msgs::Point f_next3;
//    f_next3.x = H_footholds.at(j+1).p.x();
//    f_next3.y = H_footholds.at(j+1).p.y();
//    f_next3.z = H_footholds.at(j+1).p.z();


    // publish to rviz
    visualization_msgs::Marker marker_msg;
    marker_msg.type = type;
    marker_msg.action = visualization_msgs::Marker::ADD;

    marker_msg.pose.position = f_curr;
//    marker_msg.points.push_back(f_next1);
//
//    marker_msg.points.push_back(f_curr);
//    marker_msg.points.push_back(f_next2);
//
//    marker_msg.points.push_back(f_curr);
//    marker_msg.points.push_back(f_next3);


    marker_msg.header.frame_id = frame_id;  // name of the tf message that defines the location of the body with respect to the april tag
    marker_msg.header.stamp = ros::Time();
    marker_msg.ns = "my_namespace";
    marker_msg.id = i++;
//    marker_msg.lifetime = ros::Duration(10);
    marker_msg.scale.x = 0.05;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;


    std_msgs::ColorRGBA red, green, blue, yellow, white;
    red.a = green.a = blue.a = yellow.a = white.a = alpha;


    red.r = 1.0;
    green.g = 1.0;
    blue.b = 1.0;
    yellow.r = yellow.g = 1.0;
    white.b = white.g = white.r = 1.0;


    switch (H_footholds.at(j).leg) {
      case xpp::hyq::LF:
        marker_msg.color = red;
        break;
      case xpp::hyq::RF:
        marker_msg.color = green;
        break;
      case xpp::hyq::LH:
        marker_msg.color = blue;
        break;
      case xpp::hyq::RH:
        marker_msg.color = yellow;
        break;
      default:
        break;

    }

    msg.markers.push_back(marker_msg);
  }
}




std::vector<xpp::hyq::Foothold> steps_;
void FootholdCallback(const xpp_opt::FootholdSequence& H_msg)
{
  footsteps_msg_.markers.clear();
  steps_.clear();
  int num_footholds = H_msg.foothold.size();
  std::cout << "Read " << num_footholds << " new footholds: (in Horizontal frame) \n";

  for (int i=0; i<num_footholds; ++i) {

    Eigen::Vector3d f_eig;
    f_eig <<  H_msg.foothold[i].x, H_msg.foothold[i].y, H_msg.foothold[i].z;
    xpp::hyq::Foothold f(f_eig, static_cast<xpp::hyq::LegID>(H_msg.leg[i]));
    steps_.push_back(f);
  }

  AddFootholds(footsteps_msg_, steps_,frame_id);


  using namespace xpp::hyq;
  using namespace xpp::zmp;
  using namespace xpp::utils;

  double penalty_movement_x = 1.0;
  double penalty_movement_y = 5.0;
  ZmpOptimizer::WeightsXYArray weight = {{penalty_movement_x, penalty_movement_y}};

  MarginValues margins;
  margins[FRONT] = 0.1;
  margins[HIND]  = 0.1;
  margins[SIDE]  = 0.1;
  margins[DIAG]  = 0.1; // controls sidesway motion

  double swing_time = 0.6;         
  double stance_time = 0.1;


  // start position (x,y,z) of robot
  Eigen::Vector2d cog_start_p(0.0, 0.0);
  Eigen::Vector2d cog_start_v(0.0, 0.0);
  LegDataMap<Foothold> start_stance;
  start_stance[LF] = Foothold( 0.35,  0.3, 0.0, LF);
  start_stance[RF] = Foothold( 0.35, -0.3, 0.0, RF);
  start_stance[LH] = Foothold(-0.35,  0.3, 0.0, LH);
  start_stance[RH] = Foothold(-0.35, -0.3, 0.0, RH);
  std::vector<xpp::hyq::Foothold> initial_stance_for_rviz;
  initial_stance_for_rviz.push_back(start_stance[LF]);
  initial_stance_for_rviz.push_back(start_stance[RF]);
  initial_stance_for_rviz.push_back(start_stance[LH]);
  initial_stance_for_rviz.push_back(start_stance[RH]);
  AddFootholds(footsteps_msg_, initial_stance_for_rviz,frame_id, visualization_msgs::Marker::SPHERE, 0.3);

  double t_stance_initial = 1.0; //s


  double robot_height = 0.58;



  std::vector<LegID> leg_ids;
  leg_ids.clear();
  for (Foothold f : steps_) {
    leg_ids.push_back(f.leg);
    std::cout << "f: " << f << std::endl;
  }

  // set up the general attributes of the optimizer
  xpp::zmp::ZmpOptimizer zmp_optimizer;
  zmp_optimizer.ConstructSplineSequence(leg_ids, stance_time, swing_time, t_stance_initial,t_stance_initial),
  zmp_optimizer.SetupQpMatrices(cog_start_p, cog_start_v, start_stance, steps_, weight, margins, robot_height);

  Eigen::VectorXd opt_coefficients_eig = zmp_optimizer.SolveQp();
  std::vector<ZmpSpline> splines_eig = zmp_optimizer.CreateSplines(cog_start_p, cog_start_v, opt_coefficients_eig);
  SplineContainer zmp_splines_eig;
  zmp_splines_eig.AddSplines(splines_eig);

  Eigen::VectorXd opt_footholds;
  Eigen::VectorXd opt_coefficients = zmp_optimizer.SolveIpopt(opt_footholds/*opt_coefficients_eig*/);
  std::vector<ZmpSpline> splines = zmp_optimizer.CreateSplines(cog_start_p, cog_start_v, opt_coefficients);
  // get new optimized footholds from these coefficients:
  for (int i=0; i<steps_.size(); ++i) {
    int idx = 2*i;
    steps_.at(i).p << opt_footholds[idx], opt_footholds[idx+1], 0.0;
  }



  SplineContainer zmp_splines;
  zmp_splines.AddSplines(splines);

  std_msgs::ColorRGBA red, blue;
  red.a = red.r = 1.0;
  blue.a = blue.b = 1.0;

  AddFootholds(footsteps_msg_, steps_,frame_id, visualization_msgs::Marker::CUBE);
  AddTrajectory(footsteps_msg_, zmp_splines_eig, blue, frame_id);
  AddTrajectory(footsteps_msg_, zmp_splines, red, frame_id);
}




int main(int argc, char **argv)
{


  ros::init(argc, argv, "example_node");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<visualization_msgs::MarkerArray>("zmp_trajectory", 10);
  ros::Subscriber subscriber = n.subscribe("footsteps", 1000, FootholdCallback);


  using namespace xpp::hyq;
  using namespace xpp::zmp;
  using namespace xpp::utils;

//  log4cxx::PropertyConfigurator::configure("../test/log4cxx.properties");
//  log4cxx::LoggerPtr main_logger = log4cxx::Logger::getLogger("main");


  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    publisher.publish(footsteps_msg_);
  }
}


