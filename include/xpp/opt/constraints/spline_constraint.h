/**
 @file    spline_junction_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_CONSTRAINT_H_

#include <memory>
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/bound.h>
#include <xpp/opt/variables/polynomial_spline.h>

#include "composite.h"

namespace xpp {
namespace opt {


class SplineConstraint : public Primitive {
public:
  using PolynomialPtr  = std::shared_ptr<PolynomialSpline>;
//  using ContactTimePtr = std::shared_ptr<ContactTimings>;
  using DerivativeVec  = std::vector<MotionDerivative>;

  SplineConstraint (const OptVarsPtr& opt_vars,
                    const std::string& spline_id,
                    const DerivativeVec&);
  virtual ~SplineConstraint ();





protected:
  PolynomialPtr spline_;
//  ContactTimePtr contact_timings_;

  DerivativeVec derivatives_;

  int n_dim_;
};


/** @brief Sets the spline equal to @state at time @t.
 */
class SplineStateConstraint : public SplineConstraint {
public:
  SplineStateConstraint (const OptVarsPtr& opt_vars,
                         const std::string& spline_id,
                         double t,
                         const StateLinXd& state,
                         const DerivativeVec&);
  virtual ~SplineStateConstraint ();

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

private:
  double t_;
  StateLinXd state_desired_;
//  StateLinXd state_spline_;

};


/** @brief Equates the values at spline junctions.
 */
class SplineJunctionConstraint : public SplineConstraint {
public:
  SplineJunctionConstraint (const OptVarsPtr& opt_vars,
                            const std::string& spline_id,
                            const DerivativeVec&);
  virtual ~SplineJunctionConstraint ();

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

private:
  int n_junctions_;
};





} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_CONSTRAINT_H_ */
