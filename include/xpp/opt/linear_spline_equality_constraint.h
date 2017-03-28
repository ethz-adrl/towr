/**
 @file    linear_spline_equality_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUALITY_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUALITY_CONSTRAINT_H_

#include <xpp/a_linear_constraint.h>
#include "base_motion.h"

namespace xpp {
namespace opt {

class LinearSplineEqualityConstraint : public LinearEqualityConstraint {
public:
  using ComMotionPtrU = std::unique_ptr<BaseMotion>;

  LinearSplineEqualityConstraint (const BaseMotion&);
  virtual ~LinearSplineEqualityConstraint ();

  /** @brief Updates the values of the optimization variables. */
  void UpdateVariables(const OptimizationVariables*) override;
  /** @brief Returns the Jacobian Matrix of this constraint. */
  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  ComMotionPtrU com_motion_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUALITY_CONSTRAINT_H_ */
