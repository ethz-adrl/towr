/**
 @file    cost_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Declares the class SoftConstraint
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SOFT_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SOFT_CONSTRAINT_H_

#include <memory>
#include <string>
#include <Eigen/Dense>

#include "cost.h"

namespace xpp {
namespace opt {

class Constraint;

/** Converts a constraint to a cost by weighing the quadratic violations.
  *
  * Let constraint g(x) \in R^m with upper bound b_u and lower bound b_l.
  * Let g'(x) = g(x) - 0.5(b_u+b_l) = g(x) - b
  * And it's derivative dg'(x)/dx = J(x).
  * Define a cost as c(x) = 0.5 * g'^T * W * g', where W = diag(w1,...,wm).
  * Then the gradient of the cost is defined as:
  * dc(x)/dx = (g'(x)^T * W * J)^T = J^T * W * (g(x)-b).
  */
class SoftConstraint : public Cost {
public:
  using ConstraintPtr = std::shared_ptr<Constraint>;
  using VectorXd      = Eigen::VectorXd;

  SoftConstraint (const OptVarsPtr&, const ConstraintPtr& constraint);
  virtual ~SoftConstraint ();

private:
  ConstraintPtr constraint_;
  VectorXd weights_; ///< How each constraint violation contributes to cost
  VectorXd b_;       /// average value of each upper and lower bound

  /** c(x) = 0.5 * (g-b)^T * W * (g-b)
    */
  virtual double GetCost () const override;

  /** dc(x)/dx = J^T * W * (g-b)
    */
  virtual Jacobian GetJacobian() const override;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SOFT_CONSTRAINT_H_ */
