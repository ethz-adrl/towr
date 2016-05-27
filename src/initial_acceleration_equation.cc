/**
 @file    initial_acceleration_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/zmp/initial_acceleration_equation.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

InitialAccelerationEquation::InitialAccelerationEquation (
    const Vector2d& initial_acceleration_xy,
    uint n_spline_coeff)
{
  initial_acceleration_xy_ = initial_acceleration_xy;
  n_spline_coeff_ = n_spline_coeff;
}

InitialAccelerationEquation::MatVec
InitialAccelerationEquation::BuildLinearEquation () const
{
  using namespace xpp::utils;

  int n_constraints = kDim2d *1; // acceleration in x and y direction
  MatVec lin_eq(n_constraints, n_spline_coeff_);

  int i = 0; // constraint count
  for (const Coords3D dim : Coords2DArray)
  {
    // acceleration set to zero
    int d = ContinuousSplineContainer::Index(0, dim, D);
    lin_eq.M(i,d) = 2.0;
    lin_eq.v(i++) = -initial_acceleration_xy_(dim);
  }

  assert(i==n_constraints);
  return lin_eq;
}

} /* namespace zmp */
} /* namespace xpp */
