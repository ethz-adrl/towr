/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Declares the DynamicConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_

#include "endeffectors_motion.h"
#include "endeffector_load.h"

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

  void Init(const EndeffectorsMotion&,
            const EndeffectorLoad& ee_load,
            double T, double dt);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  EndeffectorsMotion ee_motion_;
  EndeffectorLoad ee_load_;
  Eigen::VectorXd cop_;

  std::vector<double> dts_; ///< discretization of constraint

  Jacobian GetJacobianWithRespectToLambdas() const;
  Jacobian GetJacobianWithRespectToContacts() const;
  Jacobian GetJacobianWithRespectToCop() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SUPPORT_AREA_CONSTRAINT_H_ */
