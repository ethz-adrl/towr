/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include <xpp/ros/ros_visualizer.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/RobotStateTrajectory.h> // publish

#include <xpp/hyq/codegen/hyq_kinematics.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>

namespace xpp {
namespace ros {

using TrajectoryMsg = xpp_msgs::RobotStateTrajectory;
using RobotState = xpp::opt::RobotStateJoints;

static bool CheckIfInDirectoyWithIpoptConfigFile();

NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(xpp_msgs::goal_state_topic, 1,
                                &NlpOptimizerNode::UserCommandCallback, this);

  current_state_sub_ = n.subscribe(xpp_msgs::curr_robot_state,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this,
                                    ::ros::TransportHints().tcpNoDelay());

  trajectory_pub_ = n.advertise<TrajectoryMsg>(xpp_msgs::robot_trajectory_joints, 1);

  dt_ = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");

  motion_optimizer_.SetVisualizer(std::make_shared<RosVisualizer>());

  ROS_INFO_STREAM("Initialization done, waiting for current state...");
}

void
NlpOptimizerNode::CurrentStateCallback (const CurrentInfoMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg.state);
  auto fk = std::make_shared<hyq::codegen::HyQKinematics>();

  motion_optimizer_.BuildOptimizationStartState(curr_state.ConvertToCartesian(fk));
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
  motion_optimizer_.SetMotionType(static_cast<opt::MotionTypeID>(msg.motion_type));
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

  // convert to joint angles
  auto opt_traj_joints = RobotState::BuildWholeBodyTrajectory(opt_traj_cartesian,
                                  std::make_shared<hyq::HyqInverseKinematics>());

  auto msg = RosHelpers::XppToRos(opt_traj_joints);
  trajectory_pub_.publish(msg);
}

} /* namespace ros */
} /* namespace xpp */
