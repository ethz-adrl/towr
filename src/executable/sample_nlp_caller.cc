/*!
 * \file   sample_nlp_caller.cc
 * \author Alexander Winkler (winklera@ethz.ch)
 * \date   Jul 5, 2016
 * \brief  sends out a current state of hyq
 */

#include <xpp/ros/ros_conversions.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/RobotStateCartesian.h>

#include <xpp/robot_state_cartesian.h>
#include <xpp/endeffectors4.h>

using CurrentInfoMsg = xpp_msgs::RobotStateCartesian;
using Vector3d       = Eigen::Vector3d;

static const int n_ee = 4; // number of endeffectors

using namespace xpp;

int main(int argc, char **argv)
{
  ::ros::init(argc, argv, "sample_nlp_caller");

  // remove ros remapping arguments
  std::vector<std::string> argv_out;
  ::ros::removeROSArgs(argc, argv, argv_out);

  if (argv_out.size() != 7) {
    ROS_FATAL("Please specify current xy-pos/vel/acc.");
    return false;
  }

  ::ros::NodeHandle n;
  ::ros::Publisher current_info_pub = n.advertise<CurrentInfoMsg>(xpp_msgs::curr_robot_state, 10);

  // give publisher and subscriber time to register with master
  ::ros::Rate poll_rate(100);
  ROS_INFO_STREAM("Waiting for Subscriber...");
  while(::ros::ok() && current_info_pub.getNumSubscribers() == 0/*opt_params_sub.getNumPublishers() == 0 || */) {
    poll_rate.sleep(); // so node has time to connect
  }
  ROS_INFO_STREAM("Subscriber to initial state connected");

  sleep(0.5); // sleep another 0.5s to give subscriber time to listen

  RobotStateCartesian start_state_cart(n_ee);
  State3d base;
  base.lin.p_.x() = atof(argv_out[1].c_str());
  base.lin.p_.y() = atof(argv_out[2].c_str());
  base.lin.p_.z() = 0.57;
  base.lin.v_.x() = atof(argv_out[3].c_str());
  base.lin.v_.y() = atof(argv_out[4].c_str());
  base.lin.a_.x() = atof(argv_out[5].c_str()); // constraint
  base.lin.a_.y() = atof(argv_out[6].c_str()); // constraint
  start_state_cart.SetBase(base);

  EndeffectorsPos hyq_ee(n_ee);
  double start_x = 0.28;
  double start_y = 0.28;
  hyq_ee.At(kMapQuadToOpt.at(LF)) = Vector3d(base.lin.p_.x() +start_x,  base.lin.p_.y() +start_y, 0.0);
  hyq_ee.At(kMapQuadToOpt.at(RF)) = Vector3d(base.lin.p_.x() +start_x,  base.lin.p_.y() -start_y, 0.0);
  hyq_ee.At(kMapQuadToOpt.at(LH)) = Vector3d(base.lin.p_.x() -start_x,  base.lin.p_.y() +start_y, 0.0);
  hyq_ee.At(kMapQuadToOpt.at(RH)) = Vector3d(base.lin.p_.x() -start_x,  base.lin.p_.y() -start_y, 0.0);
  start_state_cart.SetEEStateInWorld(kPos, hyq_ee);

  RobotStateCartesian::ContactState contacts(n_ee);
  contacts.At(kMapQuadToOpt.at(LF)) = true;
  contacts.At(kMapQuadToOpt.at(RH)) = true;
  contacts.At(kMapQuadToOpt.at(RF)) = true;
  contacts.At(kMapQuadToOpt.at(LH)) = true;
  start_state_cart.SetContactState(contacts);

  start_state_cart.SetTime(0.0);

  CurrentInfoMsg msg = xpp::ros::RosConversions::XppToRos(start_state_cart);
  ROS_INFO_STREAM("Publishing current state...");
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
}

