/**
 @file    convexity_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_

#include "a_constraint.h"
#include "motion_structure.h"

namespace xpp {
namespace opt {

class ConvexityConstraint : public AConstraint {
public:
  ConvexityConstraint ();
  virtual ~ConvexityConstraint ();

  void Init(const MotionStructure&);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  MotionStructure motion_structure_;
  VectorXd lambdas_;
  Jacobian jac_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONVEXITY_CONSTRAINT_H_ */
