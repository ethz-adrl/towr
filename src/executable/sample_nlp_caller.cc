/*!
 * \file   sample_nlp_caller.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 5, 2016
 * \brief  sends out a current state of hyq
 */

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/CurrentInfo.h>
#include <xpp/hyq/ee_hyq.h>

using HyqState       = xpp::hyq::HyqState;
using CurrentInfoMsg = xpp_msgs::CurrentInfo;
using Vector3d       = Eigen::Vector3d;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_nlp_caller");

  // remove ros remapping arguments
  std::vector<std::string> argv_out;
  ros::removeROSArgs(argc, argv, argv_out);

  if (argv_out.size() != 7) {
    ROS_FATAL("Please specify current xy-pos/vel/acc.");
    return false;
  }

  ros::NodeHandle n;
  ros::Publisher current_info_pub = n.advertise<CurrentInfoMsg>(xpp_msgs::curr_robot_state, 1);

  // give publisher and subscriber time to register with master
  ros::Rate poll_rate(100);
  ROS_INFO_STREAM("Waiting for Subscriber...");
  while(ros::ok() && current_info_pub.getNumSubscribers() == 0/*opt_params_sub.getNumPublishers() == 0 || */) {
    poll_rate.sleep(); // so node has time to connect
  }
  ROS_INFO_STREAM("Subscriber to initial state connected");

  sleep(0.5); // sleep another 0.5s to give subscriber time to listen

  HyqState start_state;
  start_state.base_.lin.p.x() = atof(argv_out[1].c_str());
  start_state.base_.lin.p.y() = atof(argv_out[2].c_str());
  start_state.base_.lin.p.z() = 0.57;
  start_state.base_.lin.v.x() = atof(argv_out[3].c_str());
  start_state.base_.lin.v.y() = atof(argv_out[4].c_str());
  start_state.base_.lin.a.x() = atof(argv_out[5].c_str()); // constraint
  start_state.base_.lin.a.y() = atof(argv_out[6].c_str()); // constraint


  using namespace xpp;
  utils::EEXppPos hyq_ee(4);
  hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::LF)) = Vector3d( 0.359692,   0.327653, 0.0);
  hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::RF)) = Vector3d( 0.359694,  -0.327644, 0.0);
  hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::LH)) = Vector3d(-0.258797,   0.327698, 0.0);
  hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::RH)) = Vector3d(-0.358802,  -0.327695, 0.0);



//  HyqState::PosEE endeffector_W(4);
//  endeffector_W.at(kMapHyqToOpt.at(LF)) = Vector3d( 0.359692,   0.327653, 0.0);
//  endeffector_W.at(kMapHyqToOpt.at(RF)) = Vector3d( 0.359694,  -0.327644, 0.0);
//  endeffector_W.at(kMapHyqToOpt.at(LH)) = Vector3d(-0.258797,   0.327698, 0.0);
//  endeffector_W.at(kMapHyqToOpt.at(RH)) = Vector3d(-0.358802,  -0.327695, 0.0);

  start_state.SetJointAngles(hyq_ee);//endeffector_W);
//  start_state.qd[iit::HyQ::LH_KFE] = -10;
//  start_state.swingleg_.fill(false);
  start_state.swingleg_.At(hyq::kMapHyqToOpt.at(hyq::LF)) = false; // this should then also be different
  CurrentInfoMsg msg;
  msg.state = xpp::ros::RosHelpers::XppToRos(start_state);
  msg.reoptimize = false;
  ROS_INFO_STREAM("Publishing current state...");
  current_info_pub.publish(msg);
}

