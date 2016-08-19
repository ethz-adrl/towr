/*
 * zero_moment_point.cpp
 *
 *  Created on: Apr 30, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zero_moment_point.h>

namespace xpp {
namespace zmp {


ZeroMomentPoint::Vector2d
ZeroMomentPoint::CalcZmp(const State3d& cog, double height)
{
  double z_acc = cog.a.z(); // TODO: calculate z_acc based on foothold height
  Vector2d zmp = cog.Get2D().p - height/(gravity_+z_acc) * cog.Get2D().a;
  return zmp;
}


ZeroMomentPoint::VecScalar
ZeroMomentPoint::CalcZmp(const VecScalar& pos, const VecScalar& acc, double height)
{
  const double z_acc = 0.0; // TODO: calculate z_acc based on foothold height
  return pos - height/(gravity_+z_acc)*acc;
}


ZeroMomentPoint::MatVec
ZeroMomentPoint::ExpressZmpThroughCoefficients(const ContinuousSplineContainer& spline_structure,
                                               double height, Coords dim)
{
  int num_nodes = spline_structure.GetTotalNodes();
  int coeff = spline_structure.GetTotalFreeCoeff();

  MatVec zmp(num_nodes, coeff);

  int n = 0; // node counter
  for (double t_global : spline_structure.GetDiscretizedGlobalTimes())
  {
    double t_local = spline_structure.GetLocalTime(t_global);
    int spline = spline_structure.GetSplineID(t_global);

    VecScalar pos = spline_structure.ExpressComThroughCoeff(utils::kPos, t_local, spline, dim);
    VecScalar acc = spline_structure.ExpressComThroughCoeff(utils::kAcc, t_local, spline, dim);

    zmp.WriteRow(CalcZmp(pos, acc, height), n++);
  }

  assert(n<=num_nodes); // check that Eigen matrix didn't overflow
  return zmp;
}





} /* namespace zmp */
} /* namespace xpp */
