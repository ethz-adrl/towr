/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the interface to the ROS node that initializes/calls the NLP optimizer.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/ros/optimization_visualizer.h>
#include <xpp/hyq/step_sequence_planner.h>
#include <xpp/hyq/hyq_spliner.h>
#include <xpp/hyq/hyq_state.h>
#include <xpp/opt/nlp_facade.h>

#include <xpp_msgs/HyqStateJoints.h>         // receive

namespace xpp {
namespace ros {

class NlpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::opt::NlpFacade NlpFacade;

  using WholeBodyMapper = xpp::hyq::HyqSpliner;
  using OptVisualizerPtr = std::shared_ptr<OptimizationVisualizer>;
  using StepSequencePlanner = xpp::hyq::StepSequencePlanner;
  using HyqStateJoints = xpp::hyq::HyqState;
  using HyqStateJointsMsg = xpp_msgs::HyqStateJoints;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
  StepSequencePlanner step_sequence_planner_;
  NlpFacade nlp_facade_;
  WholeBodyMapper whole_body_mapper_;

  //inv_dyn this is hyq specific, bad
  HyqStateJoints curr_state_;


  virtual void OptimizeTrajectory() override final;
  virtual void PublishTrajectory() const override final;
  void CurrentStateCallback(const HyqStateJointsMsg& msg);

  double max_step_length_;
  double dt_zmp_;
  xpp::hyq::MarginValues supp_polygon_margins_;

  ::ros::Subscriber current_state_sub_;
  ::ros::Publisher trajectory_pub_hyqjoints_;

  OptVisualizerPtr optimization_visualizer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
