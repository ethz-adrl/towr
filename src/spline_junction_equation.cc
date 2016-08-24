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
    const ComSpline4& splines)
    :splines_(splines)
{}

SplineJunctionEquation::MatVec
SplineJunctionEquation::BuildLinearEquation () const
{
  using namespace xpp::utils;

  // jerk ensures that minimum acceleration cost function cannot trick
  // by moving all the big jumps in acc to the junction to save cost.
  std::vector<PosVelAcc> derivative = {kAcc, kJerk};

  int id_last = splines_.GetLastPolynomial().GetId();
  int n_constraints = derivative.size() * id_last * kDim2d;
  int n_spline_coeff = splines_.GetTotalFreeCoeff();
  MatVec junction(n_constraints, n_spline_coeff);

  int i = 0; // constraint count
  for (int id = 0; id < id_last; ++id) {
    double T = splines_.GetPolynomial(id).GetDuration();
    for (auto dim : Coords2DArray) {
      for (auto pva :  derivative) {
        VecScalar curr = splines_.ExpressComThroughCoeff(pva, T, id,   dim);
        VecScalar next = splines_.ExpressComThroughCoeff(pva, 0, id+1, dim);
        junction.WriteRow(curr-next, i++);
      }
    }
  }
  assert(i==n_constraints);

  return junction;
}

} /* namespace zmp */
} /* namespace xpp */
