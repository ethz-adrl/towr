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

LinearInvertedPendulum::LinearInvertedPendulum (double mass)
{
  m_ = mass;
}

LinearInvertedPendulum::~LinearInvertedPendulum ()
{
}

void
LinearInvertedPendulum::SetCurrent (const ComPos& com_pos,
                                    const EELoad& ee_load,
                                    const EEPos& ee_pos)
{
  pos_     = com_pos;

  // zmp_ for now keep constance, because derivative w.r.t com_acc_xy is then very tricky
  // will loose dependency on com position anyway with new model :)
  h_       = 0.58;//com_pos.z(); // assuming ee_pos are at zero
  ee_load_ = ee_load;
  ee_pos_  = ee_pos;
}

LinearInvertedPendulum::ComAcc
LinearInvertedPendulum::GetAcceleration () const
{
  Cop u = CalculateCop();
  ComAcc acc_com;
  acc_com.topRows<kDim2d>() = kGravity/h_*(pos_.topRows<kDim2d>()- u); // inverted pendulum
  acc_com.z()               = 1./m_*GetLoadSum() - kGravity;

  return acc_com;
}

LinearInvertedPendulum::JacobianRow
LinearInvertedPendulum::GetJacobianOfAccWrtBase (
    const BaseMotion& com_motion, double t, Coords3D dim) const
{
  JacobianRow com_jac     = com_motion.GetJacobian(t, kPos, dim);
  JacobianRow jac_wrt_com = kGravity/h_*com_jac;

  // z acceleration only depends on endeffector forces, not base
  if (dim == Z)
    jac_wrt_com.setZero();

  return jac_wrt_com;
}

double
LinearInvertedPendulum::GetDerivativeOfAccWrtLoad (EndeffectorID ee,
                                                   Coords3D dim) const
{
  double deriv = 0.0;

  if (dim == X || dim == Y) {
    double cop_wrt_load = GetDerivativeOfCopWrtLoad(ee)(dim);
    deriv = kGravity/h_ * (-1* cop_wrt_load);
  } else if  (dim == Z) {
    deriv = 1./m_;
  }

  return deriv;
}

double
LinearInvertedPendulum::GetDerivativeOfAccWrtEEPos (EndeffectorID ee,
                                                    Coords3D dim) const
{
  double deriv = 0.0;

  if (dim == X || dim == Y) {
    double cop_wrt_ee = GetDerivativeOfCopWrtEEPos(ee);
    deriv = kGravity/h_ * (-1* cop_wrt_ee);
  }

  // no dependency on feet, as i am not yet concerned about the moment

  return deriv;
}

double
LinearInvertedPendulum::GetDerivativeOfCopWrtEEPos (EndeffectorID ee) const
{
  return ee_load_.At(ee)/GetLoadSum();
}

LinearInvertedPendulum::Cop
LinearInvertedPendulum::GetDerivativeOfCopWrtLoad (EndeffectorID ee) const
{
  Vector2d p = ee_pos_.At(ee).topRows<kDim2d>();
  Vector2d u = CalculateCop();

  return (p - u)/GetLoadSum();
}

LinearInvertedPendulum::Cop
LinearInvertedPendulum::CalculateCop () const
{
  Cop cop = Cop::Zero();

  double sum = GetLoadSum();

  for (auto ee : ee_pos_.GetEEsOrdered()) {
    double load_percent = ee_load_.At(ee)/sum;
    cop += load_percent*ee_pos_.At(ee).topRows<kDim2d>();
  }

  return cop;
}

double
LinearInvertedPendulum::GetLoadSum () const
{
  double sum = 0.0;
  for (double load : ee_load_.ToImpl())
    sum += load;

  assert(sum > 0); // while using inverted pendulum, this must be the case.

  return sum;
}


} /* namespace opt */
} /* namespace xpp */
