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

  auto motion_params = std::make_shared<opt::quad::QuadrupedMotionParameters>();
  motion_optimizer_.SetMotionParameters(motion_params);
  motion_optimizer_.BuildDefaultStartStance();

  dt_ = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");
}

void
NlpOptimizerNode::CurrentStateCallback (const StateMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg);
  motion_optimizer_.SetInitialState(curr_state);

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
//  auto goal_prev = motion_optimizer_.goal_geom_;
  motion_optimizer_.final_base_.lin = RosHelpers::RosToXpp(msg.goal_lin);
  motion_optimizer_.final_base_.ang = RosHelpers::RosToXpp(msg.goal_ang);

  auto motion_id = static_cast<opt::MotionTypeID>(msg.motion_type);
  auto params = opt::quad::QuadrupedMotionParameters::MakeMotion(motion_id);
  motion_optimizer_.SetMotionParameters(params);

  solver_type_ = msg.use_solver_snopt ? opt::Snopt : opt::Ipopt;

  Eigen::Vector3d vel_dis(msg.vel_disturbance.x, msg.vel_disturbance.y, msg.vel_disturbance.z);
  motion_optimizer_.inital_base_.lin.v_ += vel_dis;

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
}

} /* namespace ros */
} /* namespace xpp */
