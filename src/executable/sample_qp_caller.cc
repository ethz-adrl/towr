/*!
 * \file   sample_qp_caller.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 5, 2016
 * \brief  sends out a sample current state and footholds to the QP server
 */

#include <xpp/ros/zmp_publisher.h>
#include <xpp/ros/ros_helpers.h>

#include <xpp_opt/RequiredInfoQp.h>          // send
#include <xpp_opt/OptimizedParametersQp.h>   // receive

typedef xpp_opt::RequiredInfoQp ReqInfoMsg;
typedef xpp_opt::OptimizedParametersQp OptimizedParametersMsg;

xpp::zmp::SplineContainer::VecSpline splines;
std::vector<xpp::hyq::Foothold> footholds;
void OptParamsCallback(const OptimizedParametersMsg& msg)
{
  splines = xpp::ros::RosHelpers::RosToXpp(msg.splines);
}


int main(int argc, char **argv)
{
  using namespace xpp::ros;
  if (argc==1) ROS_FATAL("Please specify current x- velocity acceleration as parameters");

  ros::init(argc, argv, "sample_qp_caller");
  ros::NodeHandle n;
  ros::Subscriber opt_params_sub = n.subscribe("optimized_parameters_qp", 1, OptParamsCallback);
  ros::Publisher current_info_pub = n.advertise<ReqInfoMsg>("required_info_qp", 1);\

  // give publisher and subscriber time to register with master
  ros::Rate poll_rate(100);
  while(opt_params_sub.getNumPublishers() == 0 || current_info_pub.getNumSubscribers() == 0) {
    poll_rate.sleep(); // so node has time to connect
  }


  ReqInfoMsg msg;
  msg.curr_state.pos.x = 0.0;
  msg.curr_state.vel.x = atof(argv[1]); // figure out why this fails
  msg.curr_state.acc.x = atof(argv[2]); // this is a constraint
//  msg.curr_state.acc.y = 1.0;

  using namespace xpp::hyq;
  xpp::hyq::LegDataMap<xpp::hyq::Foothold> start_stance;
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x,  0.3, 0.0, LF)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x, -0.3, 0.0, RF)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x,  0.3, 0.0, LH)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x, -0.3, 0.0, RH)));

  double step_length = 0.25;
  msg.steps.push_back(RosHelpers::XppToRos(Foothold(-0.35 + step_length,  0.3, 0.0, LH)));
  msg.steps.push_back(RosHelpers::XppToRos(Foothold( 0.35 + step_length,  0.3, 0.0, LF)));
  msg.steps.push_back(RosHelpers::XppToRos(Foothold(-0.35 + step_length, -0.3, 0.0, RH)));
  msg.steps.push_back(RosHelpers::XppToRos(Foothold( 0.35 + step_length, -0.3, 0.0, RF)));

  current_info_pub.publish(msg);

  xpp::ros::ZmpPublisher zmp_publisher("sample_qp_publisher");
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    zmp_publisher.AddRvizMessage(splines, // from qp server
                                 RosHelpers::RosToXpp(msg.steps),
                                 RosHelpers::RosToXpp(msg.curr_stance),
                                 0.0, 0.0,
                                 1.0);
    zmp_publisher.publish();
    loop_rate.sleep();
  }
}


