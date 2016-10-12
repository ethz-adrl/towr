/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Defines the ZeroMomentPoint class
 */

#include <xpp/zmp/zero_moment_point.h>
#include <xpp/zmp/com_motion.h>

namespace xpp {
namespace opt {

ZeroMomentPoint::ZeroMomentPoint ()
{
}

ZeroMomentPoint::ZeroMomentPoint (const ComMotion& x,
                                  const std::vector<double>& times,
                                  double height)
{
  Init(x, times, height);
}

ZeroMomentPoint::~ZeroMomentPoint ()
{
}

void
ZeroMomentPoint::Init (const ComMotion& x, const std::vector<double>& times,
                       double height)
{
  com_motion_ = x.clone();
  height_ = height;
  times_ = times;
}

ZeroMomentPoint::Vector2d
ZeroMomentPoint::CalcZmp(const State3d& cog, double height)
{
  double z_acc = cog.a.z(); // TODO: calculate z_acc based on foothold height
  Vector2d zmp = cog.Get2D().p - height/(kGravity+z_acc) * cog.Get2D().a;
  return zmp;
}

ZeroMomentPoint::Jacobian
ZeroMomentPoint::GetJacobianWrtCoeff (Coords dim) const
{
  int n_coeff = com_motion_->GetTotalFreeCoeff();

  Jacobian jac_zmp(times_.size(), n_coeff);

  int n = 0; // node counter
  for (double t_global : times_)
  {
    JacobianRow jac_pos_t = com_motion_->GetJacobian(t_global, utils::kPos, dim);
    JacobianRow jac_acc_t = com_motion_->GetJacobian(t_global, utils::kAcc, dim);

    JacobianRow zmp_t = CalcZmp(jac_pos_t, jac_acc_t, height_);
    jac_zmp.row(n++) = zmp_t;
  }

  assert(n<=times_.size()); // check that Eigen matrix didn't overflow
  return jac_zmp;
}

} /* namespace zmp */
} /* namespace xpp */


