/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the interface to the ROS node that initializes/calls the NLP optimizer.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

//#include <xpp/ros/optimizer_node_base.h>
#include <xpp/ros/optimization_visualizer.h>
//#include <xpp/hyq/step_sequence_planner.h>
//#include <xpp/hyq/hyq_spliner.h>
//#include <xpp/hyq/hyq_state.h>
//#include <xpp/opt/nlp_facade.h>

#include <xpp/opt/motion_optimizer_facade.h>

#include <xpp_msgs/HyqState.h>         // receive
#include <xpp_msgs/StateLin3d.h>       // receive

namespace xpp {
namespace ros {

class NlpOptimizerNode {
public:
  using NlpFacade           = xpp::opt::NlpFacade;
  using WholeBodyMapper     = xpp::hyq::HyqSpliner;
  using OptVisualizerPtr    = std::shared_ptr<OptimizationVisualizer>;
  using StepSequencePlanner = xpp::hyq::StepSequencePlanner;
  using HyqState            = xpp::hyq::HyqState;
  using HyqStateMsg         = xpp_msgs::HyqState;
  using StateMsg            = xpp_msgs::StateLin3d;

  using MotionOptimizer     = xpp::opt::MotionOptimizerFacade;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
//  StepSequencePlanner step_sequence_planner_;
//  NlpFacade nlp_facade_;
//  WholeBodyMapper whole_body_mapper_;

//  //inv_dyn this is hyq specific, bad
//  HyqState curr_state_;


//  void OptimizeTrajectory() override;
  void PublishTrajectory() const;
  void CurrentStateCallback(const HyqStateMsg& msg);
  void GoalStateCallback(const StateMsg& msg);

//  double max_step_length_;
//  double dt_zmp_;
//  xpp::hyq::MarginValues supp_polygon_margins_;

  ::ros::Subscriber goal_state_sub_;
  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher trajectory_pub_;

  OptVisualizerPtr optimization_visualizer_;


  MotionOptimizer motion_optimizer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
