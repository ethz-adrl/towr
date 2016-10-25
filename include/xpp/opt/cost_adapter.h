/**
 @file    cost_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Declares the class CostAdapter
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_ADAPTER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_ADAPTER_H_

#include "a_cost.h"
#include "a_constraint.h"
#include <memory>

namespace xpp {
namespace opt {

/** Converts a constraint to a cost by weighing the quadratic violations.
  *
  * Let constraint g(x) \in R^m.
  * And it's derivative dg(x)/dx = J(x).
  * Define a cost as c(x) = 0.5 * g^T * W * g, where W = diag(w1,...,wm).
  * Then the gradient of the cost is defined as:
  * dc(x)/dx = (g(x)^T * W * J)^T = J^T * W * g(x).
  */
class CostAdapter : public ACost {
public:
  using ConstraintPtr = std::shared_ptr<AConstraint>;
  using VectorXd = Eigen::VectorXd;

  CostAdapter (const ConstraintPtr& constraint);
  virtual ~CostAdapter ();

  virtual double EvaluateCost () const;
  virtual void UpdateVariables(const OptimizationVariables*);
  virtual VectorXd EvaluateGradientWrt(std::string var_set);

private:
  ConstraintPtr constraint_;
  VectorXd weights_; ///< How each constraint violation contributes to cost
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_ADAPTER_H_ */
