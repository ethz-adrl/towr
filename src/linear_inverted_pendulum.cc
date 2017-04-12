/**
 @file    linear_inverted_pendulum.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/linear_inverted_pendulum.h>
#include <vector>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

LinearInvertedPendulum::LinearInvertedPendulum ()
{
}

LinearInvertedPendulum::~LinearInvertedPendulum ()
{
}

void
LinearInvertedPendulum::SetCurrent (const ComPos& pos,
                                    double height,
                                    const EELoad& ee_load,
                                    const EEPos& ee_pos)
{
  pos_ = pos;
  h_ = height;
  ee_load_ = ee_load;
  ee_pos_ = ee_pos;
}

LinearInvertedPendulum::Cop
LinearInvertedPendulum::CalculateCop (const EELoad& ee_load,
                                      const EEPos& ee_pos) const
{
  Cop cop = Cop::Zero();

  for (auto ee : ee_pos.GetEEsOrdered())
    cop += ee_load.At(ee)*ee_pos.At(ee).topRows<kDim2d>();

  return cop;
}

LinearInvertedPendulum::ComAcc
LinearInvertedPendulum::GetAcceleration () const
{
  Cop u = CalculateCop(ee_load_, ee_pos_);
  ComAcc acc_zmp    = kGravity/h_*(pos_-u);

  return acc_zmp;
}

LinearInvertedPendulum::JacobianRow
LinearInvertedPendulum::GetJacobianWrtBase (
    const BaseMotion& com_motion, double t, Coords3D dim) const
{
  JacobianRow com_jac     = com_motion.GetJacobian(t, kPos, dim);
  JacobianRow jac_wrt_com = kGravity/h_*com_jac;

  return jac_wrt_com;
}

double
LinearInvertedPendulum::GetDerivativeOfAccWrtLoad (EndeffectorID ee,
                                                   d2::Coords dim) const
{
  double pos = ee_pos_.At(ee)(dim);
  return kGravity/h_ * (-1* pos);
}

double
LinearInvertedPendulum::GetDerivativeOfAccWrtEEPos (EndeffectorID ee) const
{
  double load = ee_load_.At(ee);
  return kGravity/h_ * (-1* load);
}

} /* namespace opt */
} /* namespace xpp */

