/**
 @file    initial_acceleration_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/zmp/initial_acceleration_equation.h>

namespace xpp {
namespace zmp {

InitialAccelerationEquation::InitialAccelerationEquation (
    const Vector2d& initial_acceleration_xy,
    const ComSpline4& splines)
  :initial_acceleration_xy_(initial_acceleration_xy),
   splines_(splines)
{
}

InitialAccelerationEquation::MatVec
InitialAccelerationEquation::BuildLinearEquation () const
{
  using namespace xpp::utils;

  int n_constraints = kDim2d *1; // acceleration in x and y direction
  MatVec lin_eq(n_constraints, splines_.GetTotalFreeCoeff());

  int i = 0; // constraint count
  double t = 0.0;
  int spline_id = 0;
  for (const Coords3D dim : Coords2DArray)
  {
    VecScalar acc = splines_.ExpressComThroughCoeff(kAcc, t, spline_id, dim);
    acc.s -= -initial_acceleration_xy_(dim);
    lin_eq.WriteRow(acc, i++);
  }

  assert(i==n_constraints);
  return lin_eq;
}

} /* namespace zmp */
} /* namespace xpp */
