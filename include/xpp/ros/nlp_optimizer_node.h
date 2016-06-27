/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_FACADE_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_FACADE_NODE_H_

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/zmp/nlp_facade.h>
#include <xpp/ros/optimization_visualizer.h>

#include <xpp_opt/RequiredInfoNlp.h>        // receive
#include <xpp_opt/OptimizedParametersNlp.h> // send

namespace xpp {
namespace ros {

class NlpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::zmp::NlpFacade NlpFacade;
  typedef xpp::hyq::LegID LegID;
  typedef xpp_opt::RequiredInfoNlp ReqInfoMsg;
  typedef xpp_opt::OptimizedParametersNlp OptParamMsg;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
  /** Fills the member variables opt_footholds and opt_coefficients
    */
  NlpFacade nlp_facade_;
  void UpdateCurrentState(const ReqInfoMsg& msg);
  void OptimizeTrajectory();
  void PublishOptimizedValues() const;


  /** Determines the order of the steps to take. This depends on what direction
    * the CoG is moving (e.g. an outward step might have to catch an acceleration
    * in a specific direction) and which leg is currently swinging.
    */
  std::vector<LegID> DetermineStepSequence(const State& curr_state, int curr_swingleg);
  LegID NextSwingLeg(LegID curr) const;
  std::vector<xpp::hyq::LegID> step_sequence_;
  LegID prev_swingleg_; // so the value of last optimization known, even if in support phase
  double margin_diagonal_;

  ::ros::Subscriber current_info_sub_;
  ::ros::Publisher opt_params_pub_;
  void CurrentInfoCallback(const ReqInfoMsg& msg);

  OptimizationVisualizer optimization_visualizer_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_FACADE_NODE_H_ */
