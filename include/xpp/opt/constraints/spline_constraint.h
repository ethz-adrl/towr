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
#include <xpp/opt/variables/spline.h>

#include "composite.h"

namespace xpp {
namespace opt {


/** @brief Sets the spline equal to @state at time @t.
 */
class SplineStateConstraint  : public Primitive {
public:
  using DerivativeVec  = std::vector<MotionDerivative>;
  using PolyPtr        = std::shared_ptr<PolynomialVars>;

  SplineStateConstraint (const OptVarsPtr& opt_vars,
                         const Spline& spline,
                         double t_global,
                         const StateLinXd& state,
                         const DerivativeVec&);
  virtual ~SplineStateConstraint ();

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

private:
  double t_global_;
  StateLinXd state_desired_;
  DerivativeVec derivatives_;
  int n_dim_;
  Spline spline_;
};


/** @brief Equates the values at spline junctions.
 */
class SplineJunctionConstraint : public Primitive {
public:
  using DerivativeVec = std::vector<MotionDerivative>;
  using VecTimes      = std::vector<double>;

  SplineJunctionConstraint (const OptVarsPtr& opt_vars,
                            const std::string& spline_id,
                            const VecTimes& poly_durations,
                            const DerivativeVec&);
  virtual ~SplineJunctionConstraint ();

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;


  enum EvalTime {Final=0, Start=1};
  int IndexRowStart(int spline_id, EvalTime which_end, MotionDerivative dxdt) const;

private:
  Spline spline_;
  int n_junctions_;

  DerivativeVec derivatives_;
  int n_dim_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_CONSTRAINT_H_ */
