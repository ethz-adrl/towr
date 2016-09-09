/*
 * xpp_optimizer_node.h
 *
 *  Created on: Apr 21, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_FACADE_NODE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_FACADE_NODE_H_

#include <xpp/ros/optimizer_node_base.h>
#include <xpp/zmp/qp_facade.h>

#include <xpp_opt/RequiredInfoQp.h>        // receive
#include <xpp_opt/OptimizedParametersQp.h> // send
#include <xpp_opt/SolveQp.h>               // service


namespace xpp {
namespace ros {

class QpOptimizerNode : public OptimizerNodeBase {
public:
  typedef xpp::zmp::QpFacade QpOptimizer;
  typedef xpp_opt::RequiredInfoQp ReqInfoMsg;
  typedef xpp_opt::OptimizedParametersQp OptParamMsg;

  QpOptimizerNode ();
  virtual ~QpOptimizerNode () {};

private:
  QpOptimizer qp_optimizer_;
  void UpdateCurrentState(const ReqInfoMsg& msg);
  void OptimizeTrajectory();
  void PublishOptimizedValues() const;

  ::ros::Subscriber current_info_sub_;
  ::ros::Publisher opt_params_pub_;
  void CurrentInfoCallback(const ReqInfoMsg& msg);

  bool start_with_com_shift_;
};

} /* namespace ros */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ROS_QP_FACADE_NODE_H_ */
