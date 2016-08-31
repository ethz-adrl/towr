/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Defines the ZeroMomentPoint class
 */

#include <xpp/zmp/zero_moment_point.h>
#include <xpp/zmp/com_motion.h>

namespace xpp {
namespace zmp {

ZeroMomentPoint::Vector2d
ZeroMomentPoint::CalcZmp(const State3d& cog, double height)
{
  double z_acc = cog.a.z(); // TODO: calculate z_acc based on foothold height
  Vector2d zmp = cog.Get2D().p - height/(kGravity+z_acc) * cog.Get2D().a;
  return zmp;
}

ZeroMomentPoint::Jacobian
ZeroMomentPoint::GetJacobianWrtCoeff (const ComMotion& x, double height, Coords dim)
{
  auto vec_t = x.GetDiscretizedGlobalTimes();
  int n_coeff = x.GetTotalFreeCoeff();

  Jacobian jac_zmp(vec_t.size(), n_coeff);

  int n = 0; // node counter
  for (double t_global : vec_t)
  {
    // get to here sparse format
    // refactor _think about getting position and acceleration at all time T
    JacobianRow jac_pos_t = x.GetJacobian(t_global, utils::kPos, dim);
    JacobianRow jac_acc_t = x.GetJacobian(t_global, utils::kAcc, dim);

    JacobianRow zmp_t = CalcZmp(jac_pos_t, jac_acc_t, height);
    jac_zmp.row(n++) = zmp_t;
  }

  assert(n<=vec_t.size()); // check that Eigen matrix didn't overflow
  return jac_zmp;
}

ZeroMomentPoint::MatVec
ZeroMomentPoint::GetLinearApproxWrtMotionCoeff(const ComMotion& com_motion,
                                               double height, Coords dim)
{
  auto vec_t = com_motion.GetDiscretizedGlobalTimes();
  int coeff = com_motion.GetTotalFreeCoeff();

  MatVec zmp(vec_t.size(), coeff);

  int n = 0; // node counter
  for (double t_global : vec_t)
  {
    // get to here sparse format
    VecScalar pos = com_motion.GetLinearApproxWrtCoeff(t_global, utils::kPos, dim);
    VecScalar acc = com_motion.GetLinearApproxWrtCoeff(t_global, utils::kAcc, dim);

    zmp.WriteRow(CalcZmp(pos, acc, height), n++);
  }

  assert(n<=vec_t.size()); // check that Eigen matrix didn't overflow
  return zmp;
}


} /* namespace zmp */
} /* namespace xpp */

