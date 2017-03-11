/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp/opt/quadruped_motion_parameters.h>

#include <xpp_msgs/RobotStateCartesianTrajectory.h> // publish
#include <xpp_msgs/ContactVector.h>                 // publish

namespace xpp {
namespace ros {

using TrajectoryCart      = std::vector<RobotStateCartesian>;
using TrajectoryCartMsg   = xpp_msgs::RobotStateCartesianTrajectory;
using ContactvectorMsg    = xpp_msgs::ContactVector;

NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(xpp_msgs::goal_state_topic, 1,
                                &NlpOptimizerNode::UserCommandCallback, this);

  current_state_sub_ = n.subscribe(xpp_msgs::curr_robot_state,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this,
                                    ::ros::TransportHints().tcpNoDelay());

  cart_trajectory_pub_  = n.advertise<TrajectoryCartMsg>(xpp_msgs::robot_trajectory_cart, 1);
  contacts_pub_         = n.advertise<ContactvectorMsg>(xpp_msgs::contact_vector, 1);

  dt_ = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");
  ROS_INFO_STREAM("Initialization done, waiting for current state...");
}

void
NlpOptimizerNode::CurrentStateCallback (const CurrentInfoMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg.state);
  motion_optimizer_.BuildOptimizationStartState(curr_state);
  ROS_DEBUG_STREAM("Received Current State");

//  OptimizeMotion();
//  PublishTrajectory();
}

void
NlpOptimizerNode::OptimizeMotion ()
{
  try {
    motion_optimizer_.OptimizeMotion(solver_type_);
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

void
NlpOptimizerNode::UserCommandCallback(const UserCommandMsg& msg)
{
  auto goal_prev = motion_optimizer_.goal_geom_;
  motion_optimizer_.goal_geom_ = RosHelpers::RosToXpp(msg.goal);

  auto motion_id = static_cast<opt::MotionTypeID>(msg.motion_type);
  auto motion_type = opt::QuadrupedMotionParameters::MakeMotion(motion_id);
  motion_optimizer_.SetMotionType(motion_type);

  solver_type_ = msg.use_solver_snopt ? opt::Snopt : opt::Ipopt;

  Eigen::Vector3d vel_dis(msg.vel_disturbance.x, msg.vel_disturbance.y, msg.vel_disturbance.z);
  motion_optimizer_.start_geom_.GetBase().lin.v += vel_dis;

  if (!msg.replay_trajectory)
    OptimizeMotion();

  PublishTrajectory();
}

void
NlpOptimizerNode::PublishTrajectory ()
{
  auto opt_traj_cartesian = motion_optimizer_.GetTrajectory(dt_);
  auto cart_traj_msg = RosHelpers::XppToRosCart(opt_traj_cartesian);
  cart_trajectory_pub_.publish(cart_traj_msg);

  // publish also directly the optimized footholds
  auto contacts = motion_optimizer_.GetContactVec();
  contacts_pub_.publish(RosHelpers::XppToRos(contacts));
}

} /* namespace ros */
} /* namespace xpp */
