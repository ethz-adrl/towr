/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/zmp/qp_optimizer.h>

#include <xpp_opt/SolveQp.h>


namespace xpp {
namespace ros {

class QpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::zmp::QpOptimizer QpOptimizer;
  typedef xpp::hyq::Foothold Foothold;


public:
  QpOptimizerNode ();
  virtual ~QpOptimizerNode () {};


private:
  void OptimizeTrajectory();
  bool OptimizeTrajectoryService(xpp_opt::SolveQp::Request& req,
                                 xpp_opt::SolveQp::Response& res);

  QpOptimizer qp_optimizer_;
  ::ros::ServiceServer opt_srv_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_ */
