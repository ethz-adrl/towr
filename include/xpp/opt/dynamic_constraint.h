/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Declares the DynamicConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_CONSTRAINT_H_

#include "a_constraint.h"

namespace xpp {
namespace opt {

/** Ensures physical feasibility of the motion
  */
class DynamicConstraint : public AConstraint {
public:
  DynamicConstraint ();
  virtual ~DynamicConstraint ();

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_DYNAMIC_CONSTRAINT_H_ */
