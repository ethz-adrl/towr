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

using TrajectoryMsg = xpp_msgs::HyqStateTrajectory;

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


  double dt_nodes           = RosHelpers::GetDoubleFromServer("/xpp/dt_nodes");
  double des_walking_height = RosHelpers::GetDoubleFromServer("/xpp/robot_height");
  double lift_height        = RosHelpers::GetDoubleFromServer("/xpp/lift_height");
  double outward_swing      = RosHelpers::GetDoubleFromServer("/xpp/outward_swing_distance");
  double trajectory_dt      = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");

  ros_visualizer_ = std::make_shared<RosVisualizer>();

  motion_optimizer_.Init(dt_nodes ,des_walking_height,
                         lift_height, outward_swing, trajectory_dt,
                         ros_visualizer_);

  CheckIfInDirectoyWithIpoptConfigFile();

  ROS_INFO_STREAM("Initialization done, waiting for current state...");
}

void
NlpOptimizerNode::CurrentStateCallback (const CurrentInfoMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg.state);
  ros_visualizer_->VisualizeCurrentState(curr_state.base_.lin.Get2D(),
                                         curr_state.GetStanceLegsInWorld());

  motion_optimizer_.SetCurrent(curr_state);

//  if (msg.reoptimize) // only re-optimize if robot signalizes to be off track
//    OptimizeAndPublishTrajectory();
}

void
NlpOptimizerNode::GoalStateCallback(const UserCommandMsg& msg)
{
  auto goal_prev = motion_optimizer_.goal_cog_;
  motion_optimizer_.goal_cog_ = RosHelpers::RosToXpp(msg.goal);
  motion_optimizer_.t_left_   = msg.t_left;

  switch (msg.motion_type) {
    case opt::WalkID:
      motion_optimizer_.motion_type_ =  std::make_shared<opt::Walk>();
      break;
    case opt::TrottID:
      motion_optimizer_.motion_type_ =  std::make_shared<opt::Trott>();
      break;
  }

  if (goal_prev != motion_optimizer_.goal_cog_) // only reoptimize if new goal position
    OptimizeAndPublishTrajectory();

//  ROS_INFO_STREAM("Goal state set to:\n" << motion_optimizer_.goal_cog_);
//  ROS_INFO_STREAM("Time left:" << msg.t_left);
}

void
NlpOptimizerNode::OptimizeAndPublishTrajectory ()
{
  motion_optimizer_.OptimizeMotion();
  ros_visualizer_->Visualize();

  // sends this info the the walking controller
  auto msg = RosHelpers::XppToRos(motion_optimizer_.GetTrajectory());
  trajectory_pub_.publish(msg);
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
