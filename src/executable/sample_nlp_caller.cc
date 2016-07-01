/*!
 * \file   sample_nlp_caller.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 5, 2016
 * \brief  sends out a sample current state to the NLP server
 */

#include <xpp/ros/ros_helpers.h>

#include <xpp_opt/RequiredInfoNlp.h>         // send
#include <xpp_opt/OptimizedParametersNlp.h> // receive

#include <xpp/ros/marker_array_builder.h>


typedef xpp_opt::RequiredInfoNlp ReqInfoMsg;
typedef xpp_opt::OptimizedParametersNlp OptimizedParametersMsg;


xpp::zmp::SplineContainer::VecSpline splines;
std::vector<xpp::hyq::Foothold> footholds;
void OptParamsCallback(const OptimizedParametersMsg& msg)
{
  splines   = xpp::ros::RosHelpers::RosToXpp(msg.splines);
  footholds = xpp::ros::RosHelpers::RosToXpp(msg.footholds);
}


int main(int argc, char **argv)
{
  using namespace xpp::ros;
  if (argc==1) ROS_FATAL("Please specify current x-velocity & acceleration as parameter");

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
  msg.curr_state.pos.x = -0.0;
  msg.curr_state.pos.y =  0.0;
  msg.curr_state.vel.x = atof(argv[1]); // figure out why this fails
  msg.curr_state.vel.y = atof(argv[2]); // figure out why this fails
  msg.curr_state.acc.x = 0.0;// this is a constraint
  msg.curr_state.acc.y = 0.0; // this is a constraint
//  msg.curr_state.acc.y = 1.0;

  using namespace xpp::hyq;
  xpp::hyq::LegDataMap<xpp::hyq::Foothold> start_stance;
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x,  0.3+msg.curr_state.pos.y, 0.0, LF)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold( 0.35+msg.curr_state.pos.x, -0.3+msg.curr_state.pos.y, 0.0, RF)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x,  0.3+msg.curr_state.pos.y, 0.0, LH)));
  msg.curr_stance.push_back(RosHelpers::XppToRos(Foothold(-0.35+msg.curr_state.pos.x, -0.3+msg.curr_state.pos.y, 0.0, RH)));

  msg.curr_swingleg = xpp::hyq::NO_SWING_LEG;

  current_info_pub.publish(msg);


  xpp::ros::MarkerArrayBuilder marker_builder;
  double walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");
  ros::Publisher ros_publisher_ = n.advertise<visualization_msgs::MarkerArray>("example_publisher", 1);

//  ros::Rate loop_rate(10);
//  while (ros::ok()) {
//    ros::spinOnce();
////    zmp_publisher.AddRvizMessage(splines,    // from nlp server
////                                 footholds,  // from qp server
////                                 RosHelpers::RosToXpp(msg.curr_stance),
////                                 0.0, 0.0,
////                                 1.0);
//    visualization_msgs::MarkerArray msg_markers = marker_builder.BuildMsg(splines, footholds, walking_height);
//    ros_publisher_.publish(msg_markers);
//    loop_rate.sleep();
//  }
}


