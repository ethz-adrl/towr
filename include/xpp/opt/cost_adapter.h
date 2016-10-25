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

/** Converts a constraint to a cost by weighing the violations and Jacobians.
  *
  * refactor careful, only makes sense for purely positive costs. Do squaring here.
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
