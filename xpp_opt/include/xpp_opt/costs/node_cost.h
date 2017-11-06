/**
 @file    node_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 7, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_COSTS_NODE_COST_H_
#define XPP_OPT_INCLUDE_XPP_OPT_COSTS_NODE_COST_H_

#include <memory>
#include <string>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_solve/composite.h>

#include <xpp_opt/variables/node_values.h>


namespace xpp {

class NodeCost : public opt::Cost {
public:
  using Nodes = std::shared_ptr<NodeValues>;

  NodeCost (const VariablesPtr&, const std::string& nodes_id);
  virtual ~NodeCost ();

  VectorXd GetValues () const override;

private:
  void FillJacobianBlock(std::string var_set, Jacobian&) const override;
  Nodes nodes_;
  std::string node_id_;

  MotionDerivative deriv_;
  int dim_;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_COSTS_NODE_COST_H_ */
