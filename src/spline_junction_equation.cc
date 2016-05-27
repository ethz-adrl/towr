/**
 @file    spline_junction_equation.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/zmp/spline_junction_equation.h>

namespace xpp {
namespace zmp {

SplineJunctionEquation::SplineJunctionEquation (
    const ContinuousSplineContainer& splines)
    :splines_(splines)
{}

SplineJunctionEquation::MatVec
SplineJunctionEquation::BuildLinearEquation () const
{
  using namespace xpp::utils;

  // junctions {acc,jerk} since pos, vel  implied
  int n_constraints = 2 /*{acc,(jerk)}*/ * (splines_.GetSplineCount()-1) * kDim2d;
  int n_spline_coeff = splines_.GetTotalFreeCoeff();

  MatVec junction(n_constraints, n_spline_coeff);

  // FIXME maybe replace with range based loop
  int i = 0; // constraint count
  for (int s = 0; s < splines_.GetSplineCount()-1; ++s)
  {
    double duration = splines_.GetSpline(s).GetDuration();
    std::array<double,6> T_curr = utils::cache_exponents<6>(duration);
    for (const Coords3D dim : Coords2DArray)
    {
      int curr_spline = ContinuousSplineContainer::Index(s, dim, A);
      int next_spline = ContinuousSplineContainer::Index(s + 1, dim, A);

      // acceleration
      junction.M(i, curr_spline + A) = 20 * T_curr[3];
      junction.M(i, curr_spline + B) = 12 * T_curr[2];
      junction.M(i, curr_spline + C) = 6  * T_curr[1];
      junction.M(i, curr_spline + D) = 2;
      junction.M(i, next_spline + D) = -2.0;
      junction.v(i++) = 0.0;


      // FIXME also increase number of constraints at top if commenting this back in
      // jerk (derivative of acceleration)
      // this ensures that minimum acceleration cost function cannot trick
      // by moving all the big jumps in acc to the junction to save cost.
      junction.M(i, curr_spline + A) = 60 * T_curr[2];
      junction.M(i, curr_spline + B) = 24 * T_curr[1];
      junction.M(i, curr_spline + C) = 6;
      junction.M(i, next_spline + C) = -6.0;
      junction.v(i++) = 0.0;
    }
  }
  assert(i==n_constraints);
  return junction;
}

} /* namespace zmp */
} /* namespace xpp */
