/**
 @file    linear_inverted_pendulum.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/lip_model.h>

#include <vector>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

LIPModel::LIPModel ()
{
}

LIPModel::~LIPModel ()
{
}

LIPModel::BaseAcc
LIPModel::GetBaseAcceleration () const
{
  BaseAcc acc;
  acc.segment(AX, 3).setZero();
  acc.segment(LX, 3) = GetLinearAcceleration();

  return acc;
}

LIPModel::ComLinAcc
LIPModel::GetLinearAcceleration () const
{
  Cop u = CalculateCop();
  ComLinAcc acc_com;
  acc_com.topRows<kDim2d>() = kGravity/h_*(com_pos_.topRows<kDim2d>()- u); // inverted pendulum
  // moved gravity to bounds, as this is constant and would mess up SNOPT
  acc_com.z()               = 1./m_*GetLoadSum();// - kGravity;

  return acc_com;
}


Jacobian
LIPModel::GetJacobianOfAccWrtBase (const BaseMotion& com_motion, double t) const
{
  Jacobian jac_wrt_com(kDim6d, com_motion.GetRows());

  for (auto dim : {LX, LY}) {
    JacobianRow com_jac     = com_motion.GetJacobian(t, kPos, dim);
    jac_wrt_com.row(dim) = kGravity/h_*com_jac;
  }

  return jac_wrt_com;
}

Jacobian
LIPModel::GetJacobianofAccWrtForce (const EndeffectorsForce& ee_force, double t,
                                    EndeffectorID ee) const
{
  Jacobian jac(kDim6d, ee_force.GetRows());

  for (auto dim : {LX, LY, LZ}) {
    double da_df = GetDerivativeOfAccWrtLoad(ee, To3D(dim));
    jac.row(dim) = da_df*ee_force.GetJacobian(t, ee, Z);
  }

  return jac;
}

Jacobian
LIPModel::GetJacobianofAccWrtEEPos (const EndeffectorsMotion& ee_motion, double t_global,
                                     EndeffectorID ee) const
{
  Jacobian jac(kDim6d, ee_motion.GetRows());

  // no dependency of CoM acceleration on height of footholds yet
  for (auto dim : {LX, LY}) {
    double deriv_ee = GetDerivativeOfAccWrtEEPos(ee, To3D(dim));
    jac.row(dim) = deriv_ee* ee_motion.GetJacobianPos(t_global, ee, To2D(dim));
  }

  return jac;
}

double
LIPModel::GetDerivativeOfAccWrtLoad (EndeffectorID ee,
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
LIPModel::GetDerivativeOfAccWrtEEPos (EndeffectorID ee,
                                                    Coords3D dim) const
{
  double deriv = 0.0;

  if (dim == X || dim == Y) {
    double cop_wrt_ee = ee_force_.At(ee).z()/GetLoadSum();
    deriv = kGravity/h_ * (-1* cop_wrt_ee);
  }

  // no dependency on feet, as i am not yet concerned about the moment

  return deriv;
}

LIPModel::Cop
LIPModel::GetDerivativeOfCopWrtLoad (EndeffectorID ee) const
{
  Vector2d p = ee_pos_.At(ee).topRows<kDim2d>();
  Vector2d u = CalculateCop();

  return (p - u)/GetLoadSum();
}

LIPModel::Cop
LIPModel::CalculateCop () const
{
  Cop cop = Cop::Zero();

  double sum = GetLoadSum(); // just in case all forces are zero (flight phase)

  for (auto ee : ee_pos_.GetEEsOrdered()) {
    double load_percent = ee_force_.At(ee).z()/sum;
    cop += load_percent*ee_pos_.At(ee).topRows<kDim2d>();
  }

  return cop;
}

double
LIPModel::GetLoadSum () const
{
  double sum = 0.0;
  for (Vector3d load : ee_force_.ToImpl())
    sum += load.z();

  sum += 1e-5; // just in case all forces are zero (flight phase)

  assert(sum > 0); // while using inverted pendulum, this must be the case.
  return sum;
}


} /* namespace opt */
} /* namespace xpp */
