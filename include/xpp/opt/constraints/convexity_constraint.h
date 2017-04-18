/**
 @file    convexity_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_

#include <memory>

#include <xpp/constraint.h>

namespace xpp {
namespace opt {

class EndeffectorLoad;

/** Enforces that the sum of all lambda variables at every discrete time = 1.
  *
  * This is a part of the constraints to represent a convex hull.
  * E.g. for a quadruped:
  * g[t_k] = lambda_LF + lambda_RF + lambda_LH + lambda_RF = 1.
  */
class ConvexityConstraint : public Constraint {
public:
  using LoadPtr = std::shared_ptr<EndeffectorLoad>;

  ConvexityConstraint (const OptVarsPtr& opt_vars_container);
  virtual ~ConvexityConstraint ();

  VectorXd GetConstraintValues() const override;
  void UpdateBounds () override;

private:
  LoadPtr ee_load_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_ */
