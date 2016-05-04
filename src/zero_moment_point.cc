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
  // fixme, write common block for this
  int num_nodes = spline_structure.GetTotalNodes();
  int coeff = spline_structure.GetTotalFreeCoeff();

  MatVec zmp(num_nodes, coeff);

  int n = 0; // node counter
  double t = 0.0;
  while (t < spline_structure.GetTotalTime())
  {
    double t_local = spline_structure.GetLocalTime(t);
    int spline = spline_structure.GetSplineID(t);
    VecScalar pos = spline_structure.ExpressCogPosThroughABCD(t_local, spline, dim);
    VecScalar acc = spline_structure.ExpressCogAccThroughABCD(t_local, spline, dim);

    zmp.WriteRow(CalcZmp(pos, acc, height), n++);

    t += spline_structure.dt_;
  }

  assert(n<=num_nodes); // check that Eigen matrix didn't overflow
  return zmp;
}





} /* namespace zmp */
} /* namespace xpp */
