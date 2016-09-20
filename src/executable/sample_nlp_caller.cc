/*!
 * \file   sample_nlp_caller.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 5, 2016
 * \brief  sends out a sample current state to the NLP server
 */

#include <xpp/ros/ros_helpers.h>
#include <xpp/zmp/com_spline6.h>

#include <xpp_opt/RequiredInfoNlp.h>         // send
#include <xpp_opt/OptimizedParametersNlp.h> // receive

#include <xpp/ros/marker_array_builder.h>


typedef xpp_opt::RequiredInfoNlp ReqInfoMsg;
typedef xpp_opt::OptimizedParametersNlp OptimizedParametersMsg;


xpp::zmp::ComSpline6::VecPolynomials splines;
std::vector<xpp::hyq::Foothold> footholds;
void OptParamsCallback(const OptimizedParametersMsg& msg)
{
  splines   = xpp::ros::RosHelpers::RosToXpp(msg.splines);
  footholds = xpp::ros::RosHelpers::RosToXpp(msg.footholds);
}

using namespace xpp::hyq;

int main(int argc, char **argv)
{
  using namespace xpp::ros;
  if (argc!=7) ROS_FATAL("Please specify current xy-pos/vel/acc.");

  ros::init(argc, argv, "sample_nlp_caller");
  ros::NodeHandle n;
  ros::Publisher current_info_pub = n.advertise<ReqInfoMsg>("required_info_nlp", 1);
  ros::Subscriber opt_params_sub = n.subscribe("optimized_parameters_nlp", 1, OptParamsCallback);

  // give publisher and subscriber time to register with master
  ros::Rate poll_rate(100);
  while(opt_params_sub.getNumPublishers() == 0 || current_info_pub.getNumSubscribers() == 0) {
    poll_rate.sleep(); // so node has time to connect
  }


  ReqInfoMsg msg;
  msg.curr_state.pos.x = atof(argv[1]);
  msg.curr_state.pos.y = atof(argv[2]);
  msg.curr_state.vel.x = atof(argv[3]);
  msg.curr_state.vel.y = atof(argv[4]);
  msg.curr_state.acc.x = atof(argv[5]); // constraint
  msg.curr_state.acc.y = atof(argv[6]); // constraint
//  msg.curr_state.acc.y = 1.0;

  auto start_stance = { Foothold( 0.359692 + msg.curr_state.pos.x,   0.327653, 0.0, LF),
                        Foothold( 0.359694 + msg.curr_state.pos.x,  -0.327644, 0.0, RF),
                        Foothold(-0.358797 + msg.curr_state.pos.x,   0.327698, 0.0, LH),
                        Foothold(-0.358802 + msg.curr_state.pos.x,  -0.327695, 0.0, RH)};

  for (auto f : start_stance) {
    msg.curr_stance.push_back(RosHelpers::XppToRos(f));
  }


//  using namespace xpp::hyq;
//  xpp::hyq::LegDataMap<xpp::hyq::Foothold> start_stance;
//  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x,  0.3+msg.curr_state.pos.y, 0.0, LF)));
//  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x, -0.3+msg.curr_state.pos.y, 0.0, RF)));
//  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x,  0.3+msg.curr_state.pos.y, 0.0, LH)));
//  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x, -0.3+msg.curr_state.pos.y, 0.0, RH)));

  msg.curr_swingleg = xpp::hyq::NO_SWING_LEG;

  current_info_pub.publish(msg);
}

