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

LinearInvertedPendulum::BaseAcc
LinearInvertedPendulum::GetBaseAcceleration () const
{
  BaseAcc acc;
  acc.segment(AX, 3).setZero();
  acc.segment(LX, 3) = GetLinearAcceleration();

  return acc;
}

LinearInvertedPendulum::ComLinAcc
LinearInvertedPendulum::GetLinearAcceleration () const
{
  Cop u = CalculateCop();
  ComLinAcc acc_com;
  acc_com.topRows<kDim2d>() = kGravity/h_*(pos_.topRows<kDim2d>()- u); // inverted pendulum
  //zmp_ take gravity out, as this is constant and messes up SNOPT
  acc_com.z()               = 1./m_*GetLoadSum() - kGravity;

  return acc_com;
}

JacobianRow
LinearInvertedPendulum::GetJacobianOfAccWrtBase (
    const BaseMotion& com_motion, double t, Coords6D dim) const
{
  JacobianRow jac_wrt_com(com_motion.GetRows());

  if (dim == LX || dim == LY) {
    JacobianRow com_jac     = com_motion.GetJacobian(t, kPos, dim);
    jac_wrt_com = kGravity/h_*com_jac;
  }

  return jac_wrt_com;
}

JacobianRow
LinearInvertedPendulum::GetJacobianofAccWrtLoad (const EndeffectorsForce& ee_force,
                                                 double t,
                                                 EndeffectorID ee,
                                                 Coords6D dim) const
{
  JacobianRow jac(ee_force.GetRows());
  // every dimension of dynamic model depends on z force
  if (dim == LX || dim == LY || dim == LZ)
    jac = GetDerivativeOfAccWrtLoad(ee, To3D(dim))*ee_force.GetJacobian(t, ee, Z);

  return jac;
}

JacobianRow
LinearInvertedPendulum::GetJacobianofAccWrtEEPos (const EndeffectorsMotion& ee_motion,
                                                  double t_global,
                                                  EndeffectorID ee,
                                                  Coords6D dim) const
{
  JacobianRow jac(ee_motion.GetRows());

  // no dependency of CoM acceleration on height of footholds yet
  if (dim == LX || dim == LY) {
    double deriv_ee = GetDerivativeOfAccWrtEEPos(ee, To3D(dim));
    jac = deriv_ee* ee_motion.GetJacobianPos(t_global, ee, To2D(dim));
  }

  return jac;
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
    double cop_wrt_ee = ee_load_.At(ee).z()/GetLoadSum();
    deriv = kGravity/h_ * (-1* cop_wrt_ee);
  }

  // no dependency on feet, as i am not yet concerned about the moment

  return deriv;
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
    double load_percent = ee_load_.At(ee).z()/sum;
    cop += load_percent*ee_pos_.At(ee).topRows<kDim2d>();
  }

  return cop;
}

double
LinearInvertedPendulum::GetLoadSum () const
{
  double sum = 0.0;
  for (Vector3d load : ee_load_.ToImpl())
    sum += load.z();

  assert(sum > 0); // while using inverted pendulum, this must be the case.

  return sum;
}


} /* namespace opt */
} /* namespace xpp */
