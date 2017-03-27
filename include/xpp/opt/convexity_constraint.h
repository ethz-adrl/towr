/**
 @file    convexity_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_

#include <xpp/a_constraint.h>
#include <xpp/opt/endeffector_load.h>

namespace xpp {
namespace opt {

/** Enforces that the sum of all lambda variables at every discrete time = 1.
  *
  * This is a part of the constraints to represent a convex hull.
  * E.g. for a quadruped:
  * g[t_k] = lambda_LF + lambda_RF + lambda_LH + lambda_RF = 1.
  */
class ConvexityConstraint : public AConstraint {
public:
  ConvexityConstraint ();
  virtual ~ConvexityConstraint ();

  /** Parametrize load by piecewise constant load values
   */
  void Init(const EndeffectorLoad&);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  EndeffectorLoad ee_load_;
  Jacobian jac_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_ */
