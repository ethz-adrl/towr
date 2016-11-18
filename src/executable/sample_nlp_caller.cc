/*!
 * \file   sample_nlp_caller.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 5, 2016
 * \brief  sends out a sample current state to the NLP server
 */

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>

#include <xpp_msgs/RequiredInfoNlp.h>
#include <xpp/ros/marker_array_builder.h>

typedef xpp_msgs::RequiredInfoNlp ReqInfoMsg;

using namespace xpp::hyq;

int main(int argc, char **argv)
{
  using namespace xpp::ros;
  ros::init(argc, argv, "sample_nlp_caller");

  // remove ros remapping arguments
  std::vector<std::string> argv_out;
  ros::removeROSArgs(argc, argv, argv_out);

  if (argv_out.size() != 7) {
    ROS_FATAL("Please specify current xy-pos/vel/acc.");
    return false;
  }

  ros::NodeHandle n;
  ros::Publisher current_info_pub = n.advertise<ReqInfoMsg>(xpp_msgs::req_info_nlp, 1);
//  ros::Subscriber opt_params_sub = n.subscribe("optimized_parameters_nlp", 1, OptParamsCallback);

  // give publisher and subscriber time to register with master
  ros::Rate poll_rate(1000);
  ROS_INFO_STREAM("Waiting for Subscriber...");
  while(ros::ok() && current_info_pub.getNumSubscribers() == 0/*opt_params_sub.getNumPublishers() == 0 || */) {
    poll_rate.sleep(); // so node has time to connect
  }
  ROS_INFO_STREAM("Subscriber to initial state connected");

  sleep(0.5); // sleep another 0.5s to give subscriber time to listen



  ReqInfoMsg msg;
  msg.curr_state.pos.x = atof(argv_out[1].c_str());
  msg.curr_state.pos.y = atof(argv_out[2].c_str());
  msg.curr_state.pos.z = 0.57;
  msg.curr_state.vel.x = atof(argv_out[3].c_str());
  msg.curr_state.vel.y = atof(argv_out[4].c_str());
  msg.curr_state.acc.x = atof(argv_out[5].c_str()); // constraint
  msg.curr_state.acc.y = atof(argv_out[6].c_str()); // constraint

  auto start_stance = { Foothold( 0.359692 + msg.curr_state.pos.x,   0.327653, 0.0, LF),
                        Foothold( 0.359694 + msg.curr_state.pos.x,  -0.327644, 0.0, RF),
                        Foothold(-0.358797 + msg.curr_state.pos.x,   0.327698, 0.0, LH),
                        Foothold(-0.358802 + msg.curr_state.pos.x,  -0.327695, 0.0, RH)};

  for (auto f : start_stance) {
    msg.curr_stance.push_back(RosHelpers::XppToRos(f));
  }

  msg.curr_swingleg = xpp::hyq::NO_SWING_LEG;

  ROS_INFO_STREAM("Publishing current state...");
  current_info_pub.publish(msg);
}

