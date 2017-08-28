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

CoeffSpline::CoeffSpline(const std::string& spline_base_id,
                         const VecTimes& poly_durations)
    :Component(-1, spline_base_id) // holds no optimization variables
{
  durations_ = poly_durations;
}

void
CoeffSpline::InitializeVariables (const VectorXd& initial_pos,
                                  const VectorXd& final_pos)
{
  double t_total = std::accumulate(durations_.begin(), durations_.end(), 0.0);
  VectorXd dp = final_pos-initial_pos;
  VectorXd average_velocity = dp/t_total;
  int num_polys = poly_vars_.size();
  for (int i=0; i<num_polys; ++i) {
    VectorXd pos = initial_pos + i/static_cast<double>(num_polys-1)*dp;
    poly_vars_.at(i)->GetPolynomial()->SetCoefficient(A, pos);
    poly_vars_.at(i)->GetPolynomial()->SetCoefficient(B, average_velocity);
  }
}


CoeffSpline::~CoeffSpline() {};

const StateLinXd
CoeffSpline::GetPoint(double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, durations_);
  return poly_vars_.at(id)->GetPolynomial()->GetPoint(t_local);
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
