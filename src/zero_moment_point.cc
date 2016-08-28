/*
 * zero_moment_point.cpp
 *
 *  Created on: Apr 30, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zero_moment_point.h>
#include <xpp/zmp/com_motion.h>

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
  double k = height/(gravity_+z_acc);
  return pos - k*acc;
}


ZeroMomentPoint::MatVec
ZeroMomentPoint::GetLinearApproxWrtMotionCoeff(const ComMotion& spline_structure,
                                               double height, Coords dim)
{
  auto vec_t = spline_structure.GetDiscretizedGlobalTimes();
  int coeff = spline_structure.GetTotalFreeCoeff();

  MatVec zmp(vec_t.size(), coeff);

  int n = 0; // node counter
  for (double t_global : vec_t)
  {
    VecScalar pos = spline_structure.GetLinearApproxWrtCoeff(t_global, utils::kPos, dim);
    VecScalar acc = spline_structure.GetLinearApproxWrtCoeff(t_global, utils::kAcc, dim);

    zmp.WriteRow(CalcZmp(pos, acc, height), n++);
  }

  assert(n<=vec_t.size()); // check that Eigen matrix didn't overflow
  return zmp;
}





} /* namespace zmp */
} /* namespace xpp */
