/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Declares the DynamicConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_

#include "motion_structure.h"
#include "eigen_std_conversions.h"

#include <xpp/a_constraint.h>
#include <memory>

namespace xpp {
namespace opt {

/** Ensures that the CoP lies within convex hull of contact points p
  *
  * At every discrete node k:
  * g_k = lambda_1*p1 + ... lambda_m*p_m - cop = 0
  */
class SupportAreaConstraint : public AConstraint {
public:
  SupportAreaConstraint ();
  virtual ~SupportAreaConstraint ();

  void Init(const MotionStructure&);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  MotionStructure motion_structure_;
  Eigen::VectorXd lambdas_;
  Eigen::VectorXd cop_;
  StdVecEigen2d footholds_;

  Jacobian GetJacobianWithRespectToLambdas() const;
  Jacobian GetJacobianWithRespectToContacts() const;
  Jacobian GetJacobianWithRespectToCop() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_ */
