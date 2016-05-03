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
  double z_acc = cog.a.z();

  Vector2d zmp;
  zmp.x() = cog.p.x() - height/(gravity_+z_acc) * cog.a.x();
  zmp.y() = cog.p.y() - height/(gravity_+z_acc) * cog.a.y();

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
                                               double height, int dim)
{
  int num_nodes = spline_structure.GetTotalNodes4ls() + spline_structure.GetTotalNodesNo4ls();
  int coeff = spline_structure.GetTotalFreeCoeff();

  MatVec zmp(num_nodes, coeff);

  int n = 0; // node counter
  for (const ZmpSpline& s : spline_structure.GetSplines()) {

    for (double i=0; i < s.GetNodeCount(spline_structure.dt_); ++i) {
      double t_local = i*spline_structure.dt_;
      VecScalar pos = spline_structure.ExpressCogPosThroughABCD(t_local, s.GetId(), dim);
      VecScalar acc = spline_structure.ExpressCogAccThroughABCD(t_local, s.GetId(), dim);
      zmp.WriteRow(CalcZmp(pos, acc, height), n++);
    }
  }

  assert(n<=num_nodes); // check that Eigen matrix didn't overflow
  return zmp;
}





} /* namespace zmp */
} /* namespace xpp */
