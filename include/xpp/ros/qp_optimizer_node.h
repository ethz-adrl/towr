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

#include <xpp_opt/RequiredInfoQp.h>        // receive
#include <xpp_opt/OptimizedParametersQp.h> // send
#include <xpp_opt/SolveQp.h>               // service



namespace xpp {
namespace ros {

class QpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::zmp::QpOptimizer QpOptimizer;
  typedef xpp_opt::RequiredInfoQp ReqInfoMsg;
  typedef xpp_opt::OptimizedParametersQp OptParamMsg;


public:
  QpOptimizerNode ();
  virtual ~QpOptimizerNode () {};


private:
  QpOptimizer qp_optimizer_;
  void OptimizeTrajectory();

  ::ros::Subscriber current_info_sub_;
  ::ros::Publisher opt_params_pub_;
  void CurrentInfoCallback(const ReqInfoMsg& msg);


  void PublishOptimizedSplines() const;

  /**
   * Service that takes in current info and returns optimzed splines
   */
  ::ros::ServiceServer opt_srv_;
  bool OptimizeTrajectoryService(xpp_opt::SolveQp::Request& req,
                                 xpp_opt::SolveQp::Response& res);
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_OPTIMIZER_NODE_H_ */
