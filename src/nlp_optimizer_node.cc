/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/HyqStateTrajectory.h> // publish

namespace xpp {
namespace ros {

static bool CheckIfInDirectoyWithIpoptConfigFile();

NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  goal_state_sub_ = n.subscribe(xpp_msgs::goal_state_topic, 1,
                                &NlpOptimizerNode::GoalStateCallback, this);

  current_state_sub_ = n.subscribe(xpp_msgs::curr_robot_state,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this);

  trajectory_pub_ = n.advertise<xpp_msgs::HyqStateTrajectory>(
      xpp_msgs::robot_trajectory_joints, 1);


  double max_step_length    = RosHelpers::GetDoubleFromServer("/xpp/max_step_length");
  double dt_zmp             = RosHelpers::GetDoubleFromServer("/xpp/dt_zmp");
  double supp_margins_diag  = RosHelpers::GetDoubleFromServer("/xpp/margin_diag");
  double t_swing            = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  double t_stance_initial   = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");
  double des_walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");
  double lift_height        = RosHelpers::GetDoubleFromServer("/xpp/lift_height");
  double outward_swing      = RosHelpers::GetDoubleFromServer("/xpp/outward_swing_distance");
  double trajectory_dt      = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");

  ros_visualizer_ = std::make_shared<RosVisualizer>();

  motion_optimizer_.Init(max_step_length,dt_zmp,supp_margins_diag,
                         t_swing, t_stance_initial,des_walking_height,
                         lift_height, outward_swing, trajectory_dt,
                         ros_visualizer_);

  CheckIfInDirectoyWithIpoptConfigFile();

  ROS_INFO_STREAM("Initialization done, waiting for current state...");
}

void
NlpOptimizerNode::CurrentStateCallback (const HyqStateMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg);
  motion_optimizer_.curr_state_ = curr_state;
  ros_visualizer_->VisualizeCurrentState(curr_state.base_.lin.Get2D(),
                                         curr_state.GetStanceLegsInWorld());
}

void
NlpOptimizerNode::GoalStateCallback(const StateMsg& msg)
{
  motion_optimizer_.goal_cog_ = RosHelpers::RosToXpp(msg);
  ROS_INFO_STREAM("Goal state set to:\n" << motion_optimizer_.goal_cog_);

  motion_optimizer_.OptimizeMotion();
  PublishTrajectory();
}

void
NlpOptimizerNode::PublishTrajectory () const
{
  // sends this info the the walking controller
  auto hyq_trajectory = motion_optimizer_.GetTrajectory();
  auto hyq_trajectory_msg = RosHelpers::XppToRos(hyq_trajectory);
  trajectory_pub_.publish(hyq_trajectory_msg);
}

/** Checks if this executable is run from where the config files for the
  * solvers are.
  */
static bool CheckIfInDirectoyWithIpoptConfigFile()
{
  char cwd[1024];
  getcwd(cwd, sizeof(cwd));
  std::string path(cwd);

  std::string ipopt_config_dir("config");
  if (path.substr( path.length() - ipopt_config_dir.length() ) != ipopt_config_dir) {
    std::string error_msg;
    error_msg = "Not run in correct directory. ";
    error_msg += "This executable has to be run from xpp_opt/" + ipopt_config_dir;
    throw ::ros::Exception(error_msg);
    return false;
  }

  return true;
}

} /* namespace ros */
} /* namespace xpp */
