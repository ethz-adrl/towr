/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>
#include <xpp/opt/com_spline.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/hyq/support_polygon_container.h>

#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>
#include <xpp_msgs/HyqStateJointsTrajectory.h>

namespace xpp {
namespace ros {

using MotionStructure = xpp::opt::MotionStructure;
using Contacts        = xpp::hyq::SupportPolygonContainer;
static bool CheckIfInDirectoyWithIpoptConfigFile();

NlpOptimizerNode::NlpOptimizerNode ()
{
  current_state_sub_ = n_.subscribe(xpp_msgs::curr_robot_state,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this);

  trajectory_pub_hyqjoints_ = n_.advertise<xpp_msgs::HyqStateJointsTrajectory>(
      xpp_msgs::robot_trajectory_joints, 1);

  supp_polygon_margins_ = xpp::hyq::SupportPolygon::GetDefaultMargins();
  supp_polygon_margins_[hyq::DIAG] = RosHelpers::GetDoubleFromServer("/xpp/margin_diag");

  max_step_length_ = RosHelpers::GetDoubleFromServer("/xpp/max_step_length");
  dt_zmp_ = RosHelpers::GetDoubleFromServer("/xpp/dt_zmp");


  // get current optimization values from the optimizer
  optimization_visualizer_ = std::make_shared<OptimizationVisualizer>();
  optimization_visualizer_->SetObserver(nlp_facade_.GetObserver());
  nlp_facade_.AttachVisualizer(optimization_visualizer_);

  CheckIfInDirectoyWithIpoptConfigFile();

  double lift_height   = RosHelpers::GetDoubleFromServer("/xpp/lift_height");
  double outward_swing = RosHelpers::GetDoubleFromServer("/xpp/outward_swing_distance");
  double trajectory_dt = RosHelpers::GetDoubleFromServer("/xpp/trajectory_dt");
  whole_body_mapper_.SetParams(0.5, lift_height, outward_swing, trajectory_dt);
  ROS_INFO_STREAM("Initialization done, waiting for current state...");
}

void
NlpOptimizerNode::CurrentStateCallback (const HyqStateJointsMsg& msg)
{
  curr_state_ = RosHelpers::RosToXpp(msg);
  // inv_dyn visualize hyq in rviz
  optimization_visualizer_->VisualizeCurrentState(curr_state_.base_.lin.Get2D(),
                                                  curr_state_.GetStanceLegsInWorld());
}

void
NlpOptimizerNode::PublishTrajectory () const
{
  // sends this info the the walking controller
  auto trajectory_hyq_joints = whole_body_mapper_.BuildWholeBodyTrajectoryJoints();
  auto msg_hyq_with_joints = xpp::ros::RosHelpers::XppToRos(trajectory_hyq_joints);
  trajectory_pub_hyqjoints_.publish(msg_hyq_with_joints);

  optimization_visualizer_->Visualize(); // sends out the footholds and com motion to rviz
}

void
NlpOptimizerNode::OptimizeTrajectory()
{
  // create the fixed motion structure
  step_sequence_planner_.Init(curr_state_.base_.lin.Get2D(), goal_cog_.Get2D(),
                              curr_state_.GetStanceLegsInWorld(),
                              des_robot_height_, max_step_length_,
                              curr_state_.SwinglegID(), supp_polygon_margins_);
  auto step_sequence        = step_sequence_planner_.DetermineStepSequence();
  bool start_with_com_shift = step_sequence_planner_.StartWithStancePhase();

  MotionStructure motion_structure;
  motion_structure.Init(curr_state_.GetStanceLegsInWorld(), step_sequence, t_swing_, t_stance_initial_,
                        start_with_com_shift, true);

  Contacts contacts;
  contacts.Init(curr_state_.GetStanceLegsInWorld(), step_sequence, supp_polygon_margins_);

  nlp_facade_.SolveNlp(curr_state_.base_.lin.Get2D(),
                       goal_cog_.Get2D(),
                       des_robot_height_,
                       motion_structure,
                       contacts,
                       dt_zmp_);

  auto& com_spline = dynamic_cast<xpp::opt::ComSpline&>(*nlp_facade_.GetMotion());
  opt_splines_   = com_spline.GetPolynomials();
  footholds_     = nlp_facade_.GetFootholds();
  motion_phases_ = nlp_facade_.GetPhases();

  whole_body_mapper_.Init(motion_phases_,opt_splines_,footholds_,
                          des_robot_height_, curr_state_);
}


/** Checks if this executable is run from where the config files for the
  * solves are.
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
