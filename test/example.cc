/*!
 * \file   example.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 4, 2014
 * \brief  An example implementation of how to generate a trajectory.
 */


#include <xpp/zmp/qp_optimizer.h>
#include <xpp/zmp/nlp_optimizer.h>
//FIXME remove this at some point
#include <xpp_opt/FootholdSequence.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include <xpp/ros/zmp_publisher.h>
#include <xpp/ros/ros_helpers.h>

#include <xpp_opt/StateLin3d.h>
#include <xpp_opt/SolveNlp.h>
#include <xpp_opt/SolveQp.h>
#include <xpp_opt/ReturnOptSplines.h>
#include <xpp_opt/ReturnOptFootholds.h>

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
}



int main(int argc, char **argv)
{

  typedef xpp_opt::SolveNlp OptService;

  ros::init(argc, argv, "xpp_example_executable");
  ros::NodeHandle n;
//  ros::Subscriber subscriber = n.subscribe("footsteps", 1000, FootholdCallback);
  xpp::ros::ZmpPublisher zmp_publisher;


  ros::ServiceClient optimizer_client = n.serviceClient<OptService>("optimize_trajectory");
  OptService srv;

  if (argc==1)
  {
    ROS_FATAL("Please specify current x-position as parameter");
  };

  srv.request.curr_state.pos.x = atof(argv[1]);
  using namespace xpp::hyq;
  xpp::hyq::LegDataMap<xpp::hyq::Foothold> start_stance;
  start_stance[LF] = Foothold( 0.35,  0.3, 0.0, LF);
  start_stance[RF] = Foothold( 0.35, -0.3, 0.0, RF);
  start_stance[LH] = Foothold(-0.35,  0.3, 0.0, LH);
  start_stance[RH] = Foothold(-0.35, -0.3, 0.0, RH);

  srv.request.curr_stance.at(0) = xpp::ros::RosHelpers::XppToRos(start_stance[LF]);
  srv.request.curr_stance.at(1) = xpp::ros::RosHelpers::XppToRos(start_stance[RF]);
  srv.request.curr_stance.at(2) = xpp::ros::RosHelpers::XppToRos(start_stance[LH]);
  srv.request.curr_stance.at(3) = xpp::ros::RosHelpers::XppToRos(start_stance[RH]);


  // this is only neccessary for qp solver
  Foothold step1(-0.35+0.25,  0.3, 0.0, LH);
  Foothold step2( 0.35+0.25,  0.3, 0.0, LF);
  Foothold step3(-0.35+0.25, -0.3, 0.0, RH);
  Foothold step4( 0.35+0.25, -0.3, 0.0, RF);

//  srv.request.steps.push_back(xpp::ros::RosHelpers::XppToRos(step1));
//  srv.request.steps.push_back(xpp::ros::RosHelpers::XppToRos(step2));
//  srv.request.steps.push_back(xpp::ros::RosHelpers::XppToRos(step3));
//  srv.request.steps.push_back(xpp::ros::RosHelpers::XppToRos(step4));



  optimizer_client.call(srv);



  // get back the optimal values
  xpp_opt::ReturnOptSplines srv_splines;
  ros::ServiceClient getter_client = n.serviceClient<xpp_opt::ReturnOptSplines>("return_optimized_splines");
  getter_client.call(srv_splines);
  std::cout << srv_splines.response.splines.size();



  xpp_opt::ReturnOptFootholds srv_footholds;
  ros::ServiceClient foothold_client = n.serviceClient<xpp_opt::ReturnOptFootholds>("return_optimized_footholds");
  foothold_client.call(srv_footholds);
  std::cout << "foothold size: " << srv_footholds.response.footholds.size();



  zmp_publisher.AddRvizMessage(xpp::ros::RosHelpers::RosToXpp(srv_splines.response.splines),
                               xpp::ros::RosHelpers::RosToXpp(srv_footholds.response.footholds),
                               0.0, 0.0, "qp", 1.0);


  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    ros::spinOnce();
    zmp_publisher.publish();
  }
}


