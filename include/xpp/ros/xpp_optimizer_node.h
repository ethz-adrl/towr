/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_XPP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_XPP_OPTIMIZER_NODE_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp_opt/StateLin3d.h>

#include <xpp/zmp/nlp_optimizer.h>

#include <ros/ros.h>

namespace xpp {
namespace ros {

class XppOptimizerNode {
public:
  typedef ::ros::Publisher Publisher;
  typedef ::ros::Subscriber Subscriber;
  typedef xpp::zmp::SplineConstraints::State State;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::zmp::NlpOptimizer NlpOptimizer;
  typedef xpp::hyq::Foothold Foothold;

public:
  XppOptimizerNode ();
  virtual
  ~XppOptimizerNode ();


private:
  void CurrentStateCallback(const xpp_opt::StateLin3d& msg);
  void GoalStateCallback(const xpp_opt::StateLin3d& msg);

  State StateLinMsgTo2DState(const xpp_opt::StateLin3d& msg) const;
  void OptimizeTrajectory() const;
  std::vector<xpp::hyq::LegID> DetermineStepSequence() const;

  Publisher opt_var_pub_;
  Subscriber curr_state_sub_;
  Subscriber goal_state_sub_;


  State goal_cog_;
  State curr_cog_;
  xpp::hyq::LegDataMap<Foothold> curr_stance_;
  double curr_execution_time_;

  NlpOptimizer nlp_optimizer_;

};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_XPP_OPTIMIZER_NODE_H_ */
