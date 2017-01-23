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
#include <xpp/hyq/joints_hyq.h>

#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/hyq/codegen/hyq_kinematics.h>
#include "../../../xpp_common/include/xpp/opt/robot_state_joints.h"

using RobotState     = xpp::opt::RobotStateJoints;
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

  RobotState start_state(xpp::hyq::kNumEE, xpp::hyq::kNumJointsPerLeg);
  RobotState::BaseState base;
  base.lin.p.x() = atof(argv_out[1].c_str());
  base.lin.p.y() = atof(argv_out[2].c_str());
  base.lin.p.z() = 0.57;
  base.lin.v.x() = atof(argv_out[3].c_str());
  base.lin.v.y() = atof(argv_out[4].c_str());
  base.lin.a.x() = atof(argv_out[5].c_str()); // constraint
  base.lin.a.y() = atof(argv_out[6].c_str()); // constraint

  start_state.SetBase(base);

  auto hyq_ik = std::make_shared<xpp::hyq::HyqInverseKinematics>();
  auto hyq_fk = std::make_shared<xpp::hyq::codegen::HyQKinematics>();

  if (false) {
    using namespace xpp;
    utils::EEXppPos hyq_ee(xpp::hyq::kNumEE);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::LF)) = Vector3d(base.lin.p.x() +0.359692,  base.lin.p.y() +0.327653, 0.0);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::RF)) = Vector3d(base.lin.p.x() +0.359694,  base.lin.p.y() -0.327644, 0.0);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::LH)) = Vector3d(base.lin.p.x() -0.358797,  base.lin.p.y() +0.327698, 0.0);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::RH)) = Vector3d(base.lin.p.x() -0.358802,  base.lin.p.y() -0.327695, 0.0);
    start_state.SetJointAngles(hyq_ee, hyq_ik);//endeffector_W);
  } else {
    using namespace xpp;
    utils::EEXppPos hyq_ee(xpp::hyq::kNumEE);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::LF)) = Vector3d(base.lin.p.x() +0.359692,  base.lin.p.y() +0.327653, 0.0);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::RF)) = Vector3d(base.lin.p.x() +0.359694,  base.lin.p.y() -0.327644, 0.0);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::LH)) = Vector3d(base.lin.p.x() -0.358797,  base.lin.p.y() +0.327698, 0.0);
    hyq_ee.At(hyq::kMapHyqToOpt.at(hyq::RH)) = Vector3d(base.lin.p.x() -0.358802,  base.lin.p.y() -0.327695, 0.0);
    start_state.SetJointAngles(hyq_ee, hyq_ik);//endeffector_W);

    RobotState::ContactState contacts(xpp::hyq::kNumEE);
    contacts.At(hyq::kMapHyqToOpt.at(hyq::LF)) = true;
    contacts.At(hyq::kMapHyqToOpt.at(hyq::RH)) = true;
    contacts.At(hyq::kMapHyqToOpt.at(hyq::RF)) = true;
    contacts.At(hyq::kMapHyqToOpt.at(hyq::LH)) = true;
    start_state.SetContactState(contacts);
  }

  start_state.SetPercentPhase(0.0);
  start_state.SetTime(0.0);

  CurrentInfoMsg msg;
  msg.state = xpp::ros::RosHelpers::XppToRos(start_state);
  ROS_INFO_STREAM("Publishing current state...");
  current_info_pub.publish(msg);
}

