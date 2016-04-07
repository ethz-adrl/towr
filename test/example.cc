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

#include <xpp/zmp/zmp_publisher.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <iostream> //std::cout, std::fixed
#include <iomanip>  //std::setprecision


std::string frame_id = "world";
visualization_msgs::MarkerArray footsteps_msg_;


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

  using namespace xpp::hyq;
  using namespace xpp::zmp;
  using namespace xpp::utils;

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

  double t_stance_initial = 1.0; //s
  double robot_height = 0.58;

  std::vector<LegID> leg_ids;
  leg_ids.clear();
  for (Foothold f : steps_) {
    leg_ids.push_back(f.leg);
    std::cout << "f: " << f << std::endl;
  }


  // create the general spline structure
  ContinuousSplineContainer trajectory;
  trajectory.Init(cog_start_p, cog_start_v,leg_ids, stance_time, swing_time, t_stance_initial,t_stance_initial);
  xpp::zmp::ZmpPublisher zmp_publisher(trajectory);

  xpp::hyq::SupportPolygonContainer supp_triangle_container;
  supp_triangle_container.Init(start_stance, steps_, margins);

  xpp::zmp::QpOptimizer zmp_optimizer(trajectory,supp_triangle_container, robot_height);
  xpp::zmp::NlpOptimizer nlp_optimizer;

  // solve QP
  Constraints::StdVecEigen2d opt_footholds_2d;
  Eigen::VectorXd opt_coefficients_eig = zmp_optimizer.SolveQp();
  zmp_publisher.AddRvizMessage(opt_coefficients_eig, steps_, "qp", 0.1);


  // solve NLP
  Eigen::VectorXd opt_coefficients = nlp_optimizer.SolveNlp(opt_footholds_2d,
                                                            trajectory,
                                                            supp_triangle_container,
                                                            opt_coefficients_eig);
  // build optimized footholds from these coefficients:
  std::vector<xpp::hyq::Foothold> footholds = steps_;
  for (uint i=0; i<footholds.size(); ++i) {
    footholds.at(i).p << opt_footholds_2d.at(i).x(), opt_footholds_2d.at(i).y(), 0.0;
  }
  zmp_publisher.AddRvizMessage(opt_coefficients, footholds, "nlp", 1.0);


  // combine the two messages
  footsteps_msg_ = zmp_publisher.zmp_msg_;
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


