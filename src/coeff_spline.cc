/**
 @file    coeff_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Defines the coeff_spline.
 */

#include <xpp/variables/coeff_spline.h>

#include <tuple>
#include <utility>

#include <xpp/cartesian_declarations.h>
#include <xpp/composite.h>
#include <xpp/polynomial.h>
#include <xpp/state.h>

namespace xpp {
namespace opt {

CoeffSpline::CoeffSpline(const OptVarsPtr& opt_vars,
                         const std::string& spline_base_id,
                         const VecTimes& poly_durations)
{
  durations_ = poly_durations;
  for (int i=0; i<poly_durations.size(); ++i) {
    auto var_set = std::dynamic_pointer_cast<PolynomialVars>(opt_vars->GetComponent(spline_base_id+std::to_string(i)));
    poly_vars_.push_back(var_set);
    polynomials_.push_back(var_set->GetPolynomial()); // links the two
  }
}

CoeffSpline::~CoeffSpline() {};

const StateLinXd
CoeffSpline::GetPoint(double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, durations_);
  return polynomials_.at(id)->GetPoint(t_local);
}

Jacobian
CoeffSpline::GetJacobian (double t_global, MotionDerivative deriv) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, durations_);
  return poly_vars_.at(id)->GetJacobian(t_local, deriv);
}

bool
CoeffSpline::DoVarAffectCurrentState(const std::string& poly_vars, double t_global) const
{
  return poly_vars == GetActiveVariableSet(t_global)->GetName();
}

CoeffSpline::VarsPtr
CoeffSpline::GetActiveVariableSet (double t_global) const
{
  auto id = GetSegmentID(t_global, durations_);
  return poly_vars_.at(id);
}


} /* namespace opt */
} /* namespace xpp */
