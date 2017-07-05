/**
 @file    spline_junction_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_JUNCTION_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_JUNCTION_CONSTRAINT_H_

#include <memory>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/bound.h>
#include <xpp/opt/variables/polynomial_spline.h>

#include "composite.h"

namespace xpp {
namespace opt {

/** @brief Equates the values at spline junctions.
 */
class SplineJunctionConstraint : public Primitive {
public:
  using DerivativeVec = std::vector<MotionDerivative>;

  SplineJunctionConstraint (const OptVarsPtr& opt_vars,
                            const std::string& spline_id,
                            const DerivativeVec&);
  virtual ~SplineJunctionConstraint ();

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

private:
  std::shared_ptr<PolynomialSpline> spline_;
  DerivativeVec derivatives_;
  int n_junctions_;
  int n_dim_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_JUNCTION_CONSTRAINT_H_ */
