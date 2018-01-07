/**
 @file    node_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 7, 2017
 @brief   Brief description
 */

#ifndef TOWR_COSTS_NODE_COST_H_
#define TOWR_COSTS_NODE_COST_H_

#include <memory>
#include <string>

#include <ifopt/composite.h>

#include <xpp_states/cartesian_declarations.h>

#include <towr/variables/node_values.h>


namespace towr {

class NodeCost : public ifopt::CostTerm {
public:
  using Nodes = std::shared_ptr<NodeValues>;

  NodeCost (const std::string& nodes_id);
  virtual ~NodeCost () = default;

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

  double GetCost () const override;

private:
  void FillJacobianBlock(std::string var_set, Jacobian&) const override;
  Nodes nodes_;
  std::string node_id_;

  xpp::MotionDerivative deriv_;
  int dim_;
};

} /* namespace towr */

#endif /* TOWR_COSTS_NODE_COST_H_ */
