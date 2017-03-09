/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Defines the ZeroMomentPoint class
 */

#include <xpp/opt/zero_moment_point.h>
#include <xpp/opt/com_motion.h>

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

ZeroMomentPoint::Jacobian
ZeroMomentPoint::GetJacobianWrtCoeff (Coords dim) const
{
  int n_coeff = com_motion_->GetTotalFreeCoeff();

  Jacobian jac_zmp(times_.size(), n_coeff);

  int n = 0; // node counter
  for (double t_global : times_)
    jac_zmp.row(n++) = GetJacobianWrtCoeff(*com_motion_, dim, height_, t_global);;


  assert(n<=times_.size()); // check that Eigen matrix didn't overflow
  return jac_zmp;
}

ZeroMomentPoint::Jacobian
ZeroMomentPoint::GetJacobianWrtCoeff (const ComMotion& com_motion, Coords dim,
                                      double height, double t)
{
  JacobianRow jac_pos_t = com_motion.GetJacobian(t, utils::kPos, dim);
  JacobianRow jac_acc_t = com_motion.GetJacobian(t, utils::kAcc, dim);

  JacobianRow jac_zmp_t = CalcZmp(jac_pos_t, jac_acc_t, height);
  return jac_zmp_t;
}


} /* namespace zmp */
} /* namespace xpp */

