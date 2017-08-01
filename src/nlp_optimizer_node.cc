/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include <kindr/Core>

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>

#include <xpp_msgs/RobotStateCartesianTrajectory.h> // publish
#include <xpp_msgs/OptParameters.h> // publish

namespace xpp {
namespace ros {


NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(xpp_msgs::goal_state_topic, 1,
                                &NlpOptimizerNode::UserCommandCallback, this);

  current_state_sub_ = n.subscribe(xpp_msgs::curr_robot_state_real,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this,
                                    ::ros::TransportHints().tcpNoDelay());

  cart_trajectory_pub_  = n.advertise<xpp_msgs::RobotStateCartesianTrajectory>
                                    (xpp_msgs::robot_trajectory_cart, 1);

  opt_parameters_pub_  = n.advertise<xpp_msgs::OptParameters>
                                    (xpp_msgs::opt_parameters, 1);

  dt_ = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");
}

void
NlpOptimizerNode::CurrentStateCallback (const StateMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg);
  SetInitialState(curr_state);

//  ROS_INFO_STREAM("Received Current Real State");
//  std::cout << curr_state.GetBase() << std::endl;
//  for (auto p : curr_state.GetEEPos().ToImpl()) {
//    std::cout << p << std::endl;
//  }

//  OptimizeMotion();
//  PublishTrajectory();
}

void
NlpOptimizerNode::OptimizeMotion ()
{
  try {
    motion_optimizer_.SolveProblem(solver_type_);
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

void
NlpOptimizerNode::UserCommandCallback(const UserCommandMsg& msg)
{
  motion_optimizer_.final_base_.lin = RosHelpers::RosToXpp(msg.goal_lin);
  motion_optimizer_.final_base_.ang = RosHelpers::RosToXpp(msg.goal_ang);
  solver_type_ = msg.use_solver_snopt ? opt::Snopt : opt::Ipopt;

  Eigen::Vector3d vel_dis(msg.vel_disturbance.x, msg.vel_disturbance.y, msg.vel_disturbance.z);
  motion_optimizer_.inital_base_.lin.v_ += vel_dis;

  PublishOptParameters();

  if (!msg.replay_trajectory)
    OptimizeMotion();

  PublishTrajectory();
}

void
NlpOptimizerNode::PublishOptParameters() const
{
  xpp_msgs::OptParameters params_msg;
  auto max_dev_xyz = motion_optimizer_.GetMotionParameters()->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = RosHelpers::XppToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = motion_optimizer_.GetMotionParameters()->GetNominalStanceInBase();
  for (auto ee : nominal_B.ToImpl())
    params_msg.nominal_ee_pos.push_back(RosHelpers::XppToRos<geometry_msgs::Point>(ee));

  params_msg.goal_lin = RosHelpers::XppToRos(motion_optimizer_.final_base_.lin);
  params_msg.goal_ang = RosHelpers::XppToRos(motion_optimizer_.final_base_.ang);

  opt_parameters_pub_.publish(params_msg);
}

void
NlpOptimizerNode::PublishTrajectory () const
{
  auto opt_traj_cartesian = motion_optimizer_.GetTrajectory(dt_);
  auto cart_traj_msg = RosHelpers::XppToRosCart(opt_traj_cartesian);
  cart_trajectory_pub_.publish(cart_traj_msg);
}

void
NlpOptimizerNode::SetInitialState (const RobotStateCartesian& initial_state)
{
  motion_optimizer_.initial_ee_W_       = initial_state.GetEEPos();

  motion_optimizer_.inital_base_ = State3dEuler(); // zero
  motion_optimizer_.inital_base_.lin = initial_state.GetBase().lin;

  kindr::RotationQuaternionD quat(initial_state.GetBase().ang.q);
  kindr::EulerAnglesZyxD euler(quat);
  euler.setUnique(); // to express euler angles close to 0,0,0, not 180,180,180 (although same orientation)
  motion_optimizer_.inital_base_.ang.p_ = euler.toImplementation().reverse();
  // assume zero euler rates and euler accelerations
}



} /* namespace ros */
} /* namespace xpp */
