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
#include <xpp/opt/nlp_facade.h>
#include <xpp_msgs/RequiredInfoNlp.h>        // receive

namespace xpp {
namespace ros {

class NlpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::opt::NlpFacade NlpFacade;
  typedef xpp::hyq::LegID LegID;
  typedef xpp_msgs::RequiredInfoNlp ReqInfoMsg;

  using WholeBodyMapper = xpp::hyq::HyqSpliner;
  using OptVisualizerPtr = std::shared_ptr<OptimizationVisualizer>;
  using StepSequencePlanner = xpp::hyq::StepSequencePlanner;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
  StepSequencePlanner step_sequence_planner_;
  NlpFacade nlp_facade_;
  WholeBodyMapper whole_body_mapper_;

  virtual void OptimizeTrajectory() override final;
  virtual void PublishTrajectory() const override final;
  void CurrentInfoCallback(const ReqInfoMsg& msg);

  double max_step_length_;
  int curr_swingleg_;
  double dt_zmp_;
  xpp::hyq::MarginValues supp_polygon_margins_;

  ::ros::Subscriber current_info_sub_;
  ::ros::Publisher trajectory_pub_hyqjoints_;


  OptVisualizerPtr optimization_visualizer_;
//  virtual void PublishOptimizedValues() const override final;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_OPTIMIZER_NODE_H_ */
