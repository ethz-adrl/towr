/*!
 * \file   example.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 4, 2014
 * \brief  An example implementation of how to generate a trajectory.
 */


#include <xpp/zmp/qp_optimizer.h>
#include <xpp/zmp/nlp_optimizer.h>

#include <xpp_opt/FootholdSequence.h>
#include <xpp_opt/FootholdSequence.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include <xpp/ros/zmp_publisher.h>
#include <xpp_opt/StateLin3d.h>
#include <xpp_opt/OptimizeTrajectory.h>
#include <xpp_opt/ReturnOptimizedTrajectory.h>
#include <xpp/ros/ros_helpers.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <iostream> //std::cout, std::fixed
#include <iomanip>  //std::setprecision



visualization_msgs::MarkerArray footsteps_msg_;


std::vector<xpp::hyq::Foothold> steps_;

void FootholdCallback(const xpp_opt::FootholdSequence& H_msg)
{
  // offset for starting point
  double x_offset = 0.0;

  footsteps_msg_.markers.clear();
  steps_.clear();
  int num_footholds = H_msg.foothold.size();
  std::cout << "Read " << num_footholds << " new footholds: (in Horizontal frame) \n";

  for (int i=0; i<num_footholds; ++i) {

    Eigen::Vector3d f_eig;
    f_eig <<  (H_msg.foothold[i].x+x_offset), H_msg.foothold[i].y, H_msg.foothold[i].z;
    xpp::hyq::Foothold f(f_eig, static_cast<xpp::hyq::LegID>(H_msg.leg[i]));
    steps_.push_back(f);
  }

  using namespace xpp::hyq;
  using namespace xpp::zmp;
  using namespace xpp::utils;


  // start position (x,y,z) of robot
  xpp::zmp::SplineConstraints::State initial_state;
  initial_state.p.x() = 0.0 + x_offset;
  LegDataMap<Foothold> start_stance;
  start_stance[LF] = Foothold( 0.35 + x_offset,  0.3, 0.0, LF);
  start_stance[RF] = Foothold( 0.35 + x_offset, -0.3, 0.0, RF);
  start_stance[LH] = Foothold(-0.35 + x_offset,  0.3, 0.0, LH);
  start_stance[RH] = Foothold(-0.35 + x_offset, -0.3, 0.0, RH);


  std::vector<LegID> leg_ids;
  leg_ids.clear();
  for (Foothold f : steps_) {
    leg_ids.push_back(f.leg);
    std::cout << "f: " << f << std::endl;
  }


  // create the general spline structure
  ContinuousSplineContainer trajectory;
  double swing_time          = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  double stance_time         = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time");
  double robot_height        = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/robot_height");
  double stance_time_initial = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");
  double stance_time_final   = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time_final");

  trajectory.Init(initial_state.p, initial_state.v, leg_ids, stance_time, swing_time, stance_time_initial,stance_time_final);
  xpp::ros::ZmpPublisher zmp_publisher(trajectory);

  xpp::hyq::SupportPolygonContainer supp_triangle_container;
  supp_triangle_container.Init(start_stance, steps_, leg_ids, SupportPolygon::GetDefaultMargins());
  zmp_publisher.AddGoal(zmp_publisher.zmp_msg_, supp_triangle_container.GetCenterOfFinalStance());
  zmp_publisher.AddStartStance(zmp_publisher.zmp_msg_,
                               supp_triangle_container.GetStartStance().ToVector(),
                               "start_stance");

  // visualize the support polygons
//  std::vector<SupportPolygon> supp_no_4l = supp_triangle_container.GetSupportPolygons();
//  for (int i=0; i<supp_no_4l.size(); ++i) {
//    zmp_publisher.AddPolygon(supp_no_4l.at(i).footholds_conv_, supp_triangle_container.GetFootholds().at(i).leg);
//  }


  // solve QP
  xpp::zmp::QpOptimizer qp_optimizer(trajectory,supp_triangle_container, robot_height);
  Eigen::VectorXd opt_coefficients_eig = qp_optimizer.SolveQp();
  zmp_publisher.AddRvizMessage(opt_coefficients_eig, steps_, 0.0, 0.0, "qp", 0.1);
  zmp_publisher.publish();


  // solve NLP
  xpp::zmp::SplineConstraints::State final_state; // zero vel,acc,jerk
  final_state.p = supp_triangle_container.GetCenterOfFinalStance();


  xpp::zmp::NlpOptimizer nlp_optimizer;
  Constraints::StdVecEigen2d opt_footholds_2d;
  Eigen::VectorXd opt_coefficients;
  nlp_optimizer.SolveNlp(initial_state,
                         final_state,
                         leg_ids,
                         start_stance,
                         opt_coefficients,
                         opt_footholds_2d,
                         opt_coefficients_eig);
  // build optimized footholds from these coefficients:
  std::vector<xpp::hyq::Foothold> footholds = steps_;
  for (uint i=0; i<footholds.size(); ++i) {
    footholds.at(i).p << opt_footholds_2d.at(i).x(), opt_footholds_2d.at(i).y(), 0.0;
  }

  // combine the two messages
  footsteps_msg_ = zmp_publisher.zmp_msg_;
}



int main(int argc, char **argv)
{


  ros::init(argc, argv, "xpp_example_executable");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<visualization_msgs::MarkerArray>("zmp_trajectory", 10);
  ros::Subscriber subscriber = n.subscribe("footsteps", 1000, FootholdCallback);


  ros::ServiceClient optimizer_client = n.serviceClient<xpp_opt::OptimizeTrajectory>("optimize_trajectory");
  ros::ServiceClient getter_client = n.serviceClient<xpp_opt::ReturnOptimizedTrajectory>("return_optimized_trajectory");
  xpp_opt::OptimizeTrajectory srv;
  srv.request.goal_state.pos.x = atof(argv[1]);
  optimizer_client.call(srv);

  xpp_opt::ReturnOptimizedTrajectory srv2;
  getter_client.call(srv2);
  std::cout << srv2.response.x.spline_coeff.size();

//  ros::Publisher goal_state_publisher = n.advertise<xpp_opt::StateLin3d>("goal_state", 10);
//  xpp_opt::StateLin3d goal_state;
//  goal_state.pos.x = 0.5;

//  ros::Rate loop_rate(100);
//  while (ros::ok()) {
//    goal_state_publisher.publish(goal_state);
//    ros::spinOnce();
    //    publisher.publish(footsteps_msg_);
//  }
}


