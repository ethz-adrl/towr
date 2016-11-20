/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>
//#include <xpp/opt/com_spline.h>
//#include <xpp/opt/motion_structure.h>
//#include <xpp/hyq/support_polygon_container.h>

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/HyqStateTrajectory.h> // publish

namespace xpp {
namespace ros {

//using MotionStructure = xpp::opt::MotionStructure;
//using Contacts        = xpp::hyq::SupportPolygonContainer;
static bool CheckIfInDirectoyWithIpoptConfigFile();

NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  goal_state_sub_ = n.subscribe(xpp_msgs::goal_state_topic, 1,
                                &NlpOptimizerNode::GoalStateCallback, this);

  motion_optimizer_.t_swing_   = RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  motion_optimizer_.t_stance_initial_   = RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");

  motion_optimizer_.des_robot_height_ = RosHelpers::GetDoubleFromServer("/xpp/robot_height");



  current_state_sub_ = n.subscribe(xpp_msgs::curr_robot_state,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this);

  trajectory_pub_ = n.advertise<xpp_msgs::HyqStateTrajectory>(
      xpp_msgs::robot_trajectory_joints, 1);

  motion_optimizer_.supp_polygon_margins_ = xpp::hyq::SupportPolygon::GetDefaultMargins();
  motion_optimizer_.supp_polygon_margins_[hyq::DIAG] = RosHelpers::GetDoubleFromServer("/xpp/margin_diag");

  motion_optimizer_.max_step_length_ = RosHelpers::GetDoubleFromServer("/xpp/max_step_length");
  motion_optimizer_.dt_zmp_ = RosHelpers::GetDoubleFromServer("/xpp/dt_zmp");


  // get current optimization values from the optimizer
  optimization_visualizer_ = std::make_shared<OptimizationVisualizer>();
  optimization_visualizer_->SetObserver(motion_optimizer_.nlp_facade_.GetObserver());
  motion_optimizer_.nlp_facade_.AttachVisualizer(optimization_visualizer_);

  CheckIfInDirectoyWithIpoptConfigFile();

  double lift_height   = RosHelpers::GetDoubleFromServer("/xpp/lift_height");
  double outward_swing = RosHelpers::GetDoubleFromServer("/xpp/outward_swing_distance");
  double trajectory_dt = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");
  motion_optimizer_.whole_body_mapper_.SetParams(0.5, lift_height, outward_swing, trajectory_dt);
  ROS_INFO_STREAM("Initialization done, waiting for current state...");
}

void
NlpOptimizerNode::CurrentStateCallback (const HyqStateMsg& msg)
{
  auto curr_state = RosHelpers::RosToXpp(msg);
  motion_optimizer_.curr_state_ = curr_state;
  // inv_dyn visualize hyq in rviz
  optimization_visualizer_->VisualizeCurrentState(curr_state.base_.lin.Get2D(),
                                                  curr_state.GetStanceLegsInWorld());
}

void
NlpOptimizerNode::GoalStateCallback(const StateMsg& msg)
{
  motion_optimizer_.goal_cog_ = RosHelpers::RosToXpp(msg);
  ROS_INFO_STREAM("Goal state set to:\n" << motion_optimizer_.goal_cog_);

  motion_optimizer_.OptimizeMotion();
//  OptimizeTrajectory();
  PublishTrajectory();
}

void
NlpOptimizerNode::PublishTrajectory () const
{
  // sends this info the the walking controller
  auto hyq_trajectory = motion_optimizer_.GetTrajectory();
  auto hyq_trajectory_msg = xpp::ros::RosHelpers::XppToRos(hyq_trajectory);
  trajectory_pub_.publish(hyq_trajectory_msg);

  optimization_visualizer_->Visualize(); // sends out the footholds and com motion to rviz
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
