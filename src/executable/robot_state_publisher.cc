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
#include <xpp/endeffectors.h>


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
  ::ros::Publisher current_info_pub = n.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_current, 10);

  // give publisher and subscriber time to register with master
  ::ros::Rate poll_rate(100);
  ROS_INFO_STREAM("Waiting for Subscriber...");
  while(::ros::ok() && current_info_pub.getNumSubscribers() == 0/*opt_params_sub.getNumPublishers() == 0 || */) {
    poll_rate.sleep(); // so node has time to connect
  }
  ROS_INFO_STREAM("Subscriber to initial state connected");

  sleep(1.0); // sleep another 0.5s to give subscriber time to listen


  int n_ee = 4;
  RobotStateCartesian state(n_ee);
  state.ee_contact_.SetAll(false);

  // linear position
  state.base_.lin.p_.x() = atof(argv_out[1].c_str());
  state.base_.lin.p_.y() = atof(argv_out[2].c_str());
  state.base_.lin.p_.z() = atof(argv_out[3].c_str());

  // angular position
  double roll  = atof(argv_out[4].c_str());
  double pitch = atof(argv_out[5].c_str());
  double yaw   = atof(argv_out[6].c_str());
  state.base_.ang.q = GetQuaternionFromEulerZYX(yaw, pitch, roll);


  // endeffector position expressed in base frame
  double x_ee_B = 0.33;
  double y_ee_B = 0.13;
  double z_ee_B = -0.46;
  Vector3d init = state.base_.lin.p_;
  state.ee_motion_.At(quad::kMapIDToEE.at(quad::LF)).p_ = Vector3d(+x_ee_B,  +y_ee_B, z_ee_B) + init;
  state.ee_motion_.At(quad::kMapIDToEE.at(quad::RF)).p_ = Vector3d(+x_ee_B,  -y_ee_B, z_ee_B) + init;
  state.ee_motion_.At(quad::kMapIDToEE.at(quad::LH)).p_ = Vector3d(-x_ee_B,  +y_ee_B, z_ee_B) + init;
  state.ee_motion_.At(quad::kMapIDToEE.at(quad::RH)).p_ = Vector3d(-x_ee_B,  -y_ee_B, z_ee_B) + init;


  state.t_global_ = 1.0;

  auto msg = xpp::ros::RosConversions::XppToRos(state);
  ROS_INFO_STREAM("Publishing current state to " << current_info_pub.getTopic());
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);
  current_info_pub.publish(msg);

//  ::ros::spin();
}

