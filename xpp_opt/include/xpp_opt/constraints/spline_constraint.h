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

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>

#include <ifopt/composite.h>

#include <xpp_opt/polynomial.h>
#include <xpp_opt/variables/coeff_spline.h>


namespace xpp {


/** @brief Sets the spline equal to @state at time @t.
 */
class SplineStateConstraint  : public opt::Constraint {
public:
  using DerivativeVec  = std::vector<MotionDerivative>;
  using PolyPtr        = std::shared_ptr<PolynomialVars>;
  using SplineT        = std::shared_ptr<Spline>;
  using Dimensions     = std::vector<Coords3D>;

  SplineStateConstraint (const std::string& id,
                         double t_global,
                         const StateLinXd& state,
                         const DerivativeVec&,
                         const Dimensions&);
  virtual ~SplineStateConstraint ();

  virtual void LinkVariables(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  double t_global_;
  StateLinXd state_desired_;
  DerivativeVec derivatives_;
  Dimensions dims_;
  SplineT spline_;
  std::string id_;
};


/** @brief Equates the values at spline junctions.
 */
class SplineJunctionConstraint : public opt::Constraint {
public:
  using DerivativeVec = std::vector<MotionDerivative>;
  using VecTimes      = std::vector<double>;
  using SplineT       = std::shared_ptr<CoeffSpline>;

  SplineJunctionConstraint (const std::string& spline_id,
                            const DerivativeVec&);
  virtual ~SplineJunctionConstraint ();

  virtual void LinkVariables(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  SplineT spline_;
  int n_junctions_;
  std::string id_;

  DerivativeVec derivatives_;
  int n_dim_;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_SPLINE_CONSTRAINT_H_ */
