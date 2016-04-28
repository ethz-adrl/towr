/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/zmp/nlp_optimizer.h>

#include <xpp_opt/RequiredInfoNlp.h>        // receive
#include <xpp_opt/OptimizedParametersNlp.h> // send



namespace xpp {
namespace ros {

class NlpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::zmp::NlpOptimizer NlpOptimizer;
  typedef xpp_opt::RequiredInfoNlp ReqInfoMsg;
  typedef xpp_opt::OptimizedParametersNlp OptParamMsg;

public:
  NlpOptimizerNode ();
  virtual ~NlpOptimizerNode () {};

private:
  /**
   * Fills the member variables opt_footholds and opt_coefficients
   */
  NlpOptimizer nlp_optimizer_;
  void OptimizeTrajectory();

  std::vector<xpp::hyq::LegID> DetermineStepSequence() const;

  ::ros::Subscriber current_info_sub_;
  ::ros::Publisher opt_params_pub_;
  void CurrentInfoCallback(const ReqInfoMsg& msg);

};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_NLP_OPTIMIZER_NODE_H_ */
