/**
 @file    coeff_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2017
 @brief   Defines the coeff_spline.
 */

#include <xpp/opt/variables/coeff_spline.h>

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

Jacobian
CoeffSpline::GetJacobian (double t_global, MotionDerivative deriv) const
{
  double t_local = GetLocalTime(t_global);
  return GetActiveVariableSet(t_global)->GetJacobian(t_local, deriv);
}

bool
CoeffSpline::DoVarAffectCurrentState(const std::string& poly_vars, double t_global) const
{
  return poly_vars == GetActiveVariableSet(t_global)->GetName();
}

CoeffSpline::VarsPtr
CoeffSpline::GetActiveVariableSet (double t_global) const
{
  int id = GetSegmentID(t_global);
  return poly_vars_.at(id);
}


} /* namespace opt */
} /* namespace xpp */
