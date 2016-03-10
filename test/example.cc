/*!
 * \file   example.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 4, 2014
 * \brief  An example implementation of how to generate a trajectory.
 */

#include <xpp/zmp/zmp_optimizer.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/basicconfigurator.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <iostream> //std::cout, std::fixed
#include <iomanip>  //std::setprecision







visualization_msgs::MarkerArray BuildRvizMessage(const std::vector<xpp::hyq::Foothold>& H_footholds,
                                                 std::string frame_id)
{
  visualization_msgs::MarkerArray footsteps_rviz_msg;

  int i = 0;
  for (const xpp::hyq::Foothold& f : H_footholds) {

    ::geometry_msgs::Point point;
    point.x = f.p.x();
    point.y = f.p.y();
    point.z = f.p.z();

    // publish to rviz
    visualization_msgs::Marker marker_msg;

    // this is the name of the tf message that defines the location of the body
    // with respect to the april tag
    marker_msg.header.frame_id = frame_id;
    marker_msg.header.stamp = ros::Time();
    marker_msg.ns = "my_namespace";
    marker_msg.id = i;
    marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.lifetime = ros::Duration(10);
//    marker_msg.pose.position.x = point.x;
//    marker_msg.pose.position.y = point.y;
//    marker_msg.pose.position.z = point.z;
    marker_msg.points.push_back(point);
    ::geometry_msgs::Point point2 = point; point2.x += 0.03;
    marker_msg.points.push_back(point2);
//    marker_msg.pose.orientation.x = 0.0;
//    marker_msg.pose.orientation.y = 0.0;
//    marker_msg.pose.orientation.z = 0.0;
//    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.005;
//    marker_msg.scale.y = 0.02;
//    marker_msg.scale.z = 0.02;


    if (i<4 || i>=H_footholds.size()-4) {
      marker_msg.color.a = 1.0; // Don't forget to set the alpha!
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
    } else {

      marker_msg.color.a = 1.0; // Don't forget to set the alpha!
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 0.0;
      marker_msg.color.b = 0.0;
      switch (f.leg) {
        case xpp::hyq::LF:
          marker_msg.color.r = 1.0;
          break;
        case xpp::hyq::RF:
          marker_msg.color.g = 1.0;
          break;
        case xpp::hyq::LH:
          marker_msg.color.b = 1.0;
          break;
        case xpp::hyq::RH:
          marker_msg.color.r = 1.0;
          marker_msg.color.g = 1.0;
          marker_msg.color.b = 1.0;
          break;
        default:
          break;
      }
    }

    i++;
    footsteps_rviz_msg.markers.push_back(marker_msg);
  }
  return footsteps_rviz_msg;
}










int main(int argc, char **argv)
{

  std::string frame_id = "world";
  ros::init(argc, argv, "zmp_publisher");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<visualization_msgs::MarkerArray>("footsteps", 10);

  using namespace xpp::hyq;
  using namespace xpp::zmp;
  using namespace xpp::utils;

  log4cxx::PropertyConfigurator::configure("../test/log4cxx.properties");
  log4cxx::LoggerPtr main_logger = log4cxx::Logger::getLogger("main");

  int splines_per_step = 2;
  int splines_per_4ls = 1;

  double penalty_movement_x = 1.0;
  double penalty_movement_y = 1.5;
  ZmpOptimizer::WeightsXYArray weight = {{penalty_movement_x, penalty_movement_y}};

  MarginValues margins;
  margins[FRONT] = 0.1;
  margins[HIND]  = 0.1;
  margins[SIDE]  = 0.1;
  margins[DIAG]  = 0.05; // controls sidesway motion

  double swing_time = 0.6;         
  double stance_time = 0.2;         


  // start position (x,y,z) of robot
  Eigen::Vector2d cog_start_p(0.0, 0.0);
  Eigen::Vector2d cog_start_v(0.0, 0.0);
  LegDataMap<Foothold> start_stance;
  start_stance[LF] = Foothold( 0.35,  0.3, 0.0, LF);
  start_stance[RF] = Foothold( 0.35, -0.3, 0.0, RF);
  start_stance[LH] = Foothold(-0.35,  0.3, 0.0, LH);
  start_stance[RH] = Foothold(-0.35, -0.3, 0.0, RH);
  double t_stance_initial = 1.0; //s

  // steps in global reference frame
  std::vector<Foothold> steps;
  steps.push_back(Foothold(-0.25,  0.3, 0.0, LH));
  steps.push_back(Foothold( 0.50,  0.3, 0.0, LF));
  steps.push_back(Foothold(-0.12, -0.3, 0.0, RH));
  steps.push_back(Foothold( 0.62, -0.3, 0.0, RF));
  steps.push_back(Foothold( 0.01,  0.3, 0.0, LH));
  steps.push_back(Foothold( 0.75,  0.3, 0.0, LF));
  steps.push_back(Foothold( 0.14, -0.3, 0.0, RH));
  steps.push_back(Foothold( 0.88, -0.3, 0.0, RF));


  visualization_msgs::MarkerArray footsteps_msg = BuildRvizMessage(steps,frame_id);


  double robot_height = 0.58;


  ////////////////// QP optimization using eigen_quadprog /////////////////////
  std::vector<LegID> leg_ids;
  for (Foothold f : steps)
    leg_ids.push_back(f.leg);

  // set up the general attributes of the optimizer
  Ipopt::SmartPtr<Ipopt::NlpIpoptZmp> my_nlp = new Ipopt::NlpIpoptZmp();
  my_nlp->zmp_optimizer_.ConstructSplineSequence(leg_ids, stance_time, swing_time, t_stance_initial,t_stance_initial),
  my_nlp->zmp_optimizer_.SetupQpMatrices(cog_start_p, cog_start_v, start_stance,
                          steps, weight, margins, robot_height);

  Eigen::VectorXd opt_coefficients = my_nlp->zmp_optimizer_.SolveQp();





  // solve the qp with ipopt instead of eigen_quadprog:

  Ipopt::IpoptApplication app;
  Ipopt::ApplicationReturnStatus status = app.Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    return (int) status;
  }
  status = app.OptimizeTNLP(my_nlp);
  if (status == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;

    // override optimized coefficients of eigen quadprog
    opt_coefficients = my_nlp->x_final_;
    std::cout << "the final coefficients are: " << opt_coefficients.transpose() << "\n";
  }




  std::vector<ZmpSpline> splines = my_nlp->zmp_optimizer_.CreateSplines(cog_start_p, cog_start_v, opt_coefficients);
  /////////////////////////////////////////////////////////////////////////////


  SplineContainer zmp_splines;
  zmp_splines.AddSplines(splines);
  LOG4CXX_INFO(main_logger, "\nZMP-optimized CoG Trajectory:\n"
               << "position(p), velocity(v), acclerations(a) [x,y]");

  int i=100;
  for (double t(0.0); t < swing_time*steps.size(); t+= 0.02)
  {
    Point2d cog_state;
    zmp_splines.GetCOGxy(t, cog_state);


    visualization_msgs::Marker marker_msg;

    geometry_msgs::Point cog;
    cog.x = cog_state.p.x();
    cog.y = cog_state.p.y();
    cog.z = 0.0;

    marker_msg.pose.position = cog;
    marker_msg.header.frame_id = frame_id;
    marker_msg.header.stamp = ros::Time();
    marker_msg.ns = "my_namespace";
    marker_msg.id = i++;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.lifetime = ros::Duration(10);
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.003;
    marker_msg.scale.y = 0.003;
    marker_msg.scale.z = 0.003;
    marker_msg.color.a = 1.0; // Don't forget to set the alpha!
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 1.0;

    footsteps_msg.markers.push_back(marker_msg);



    LOG4CXX_INFO(main_logger, "t = " << t << "s:\t"
                             << std::setprecision(2) << std::fixed
                             << cog_state );
  }







  ros::Rate loop_rate(100);
  while (ros::ok()) {
    publisher.publish(footsteps_msg);
  }
}


