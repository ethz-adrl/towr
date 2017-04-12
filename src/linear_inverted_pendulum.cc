/**
 @file    linear_inverted_pendulum.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/linear_inverted_pendulum.h>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

LinearInvertedPendulum::LinearInvertedPendulum ()
{
  // TODO Auto-generated constructor stub
}

LinearInvertedPendulum::~LinearInvertedPendulum ()
{
  // TODO Auto-generated destructor stub
}

void
LinearInvertedPendulum::SetCurrent (const ComPos& pos,
                                    double height,
                                    const EELoad& ee_load,
                                    const EEPos& ee_pos)
{
  pos_ = pos;
  h_ = height;
  cop_ = CalculateCop(ee_load, ee_pos);
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
LinearInvertedPendulum::GetDerivative (const Cop& u) const
{
//  Eigen::Array2d k1 = (pos_-u)/h_;
//  Eigen::Array2d k2 = 2*h_/(h_*h_ + (pos_-u).array().square());
//
//  ComAcc acc        = k1*(k2   *vel_.array().square() + kGravity);
//  ComAcc acc_approx = k1*(2./h_*vel_.array().square() + kGravity);
//  ComAcc acc_zmp    = k1*(                            + kGravity);

  ComAcc acc_zmp    = kGravity/h_*(pos_-u);

  return acc_zmp;
}

LinearInvertedPendulum::JacobianRow
LinearInvertedPendulum::GetJacobianApproxWrtSplineCoeff (
    const BaseMotion& com_motion, double t,
    Coords3D dim, const Cop& u) const
{
  JacobianRow jac_pos     = com_motion.GetJacobian(t, kPos, dim);

//  JacobianRow vel2    = com_motion.GetJacobianVelSquared(t,dim);
//  JacobianRow posvel2 = com_motion.GetJacobianPosVelSquared(t, dim);
//  JacobianRow jac_approx = 1./h_*(kGravity*pos + 2./h_*posvel2 - 2./h_*u(dim)*vel2);

  JacobianRow jac_zmp    = kGravity/h_*jac_pos;

  return jac_zmp;
}

double
LinearInvertedPendulum::GetDerivativeOfAccWrtCop (d2::Coords dim) const
{
//  double vel2    = std::pow(vel_(dim),2);
//  double jac_approx = 1./h_*(-kGravity -2./h_*vel2);

  double jac_zmp    = kGravity/h_*(-1);

  return jac_zmp;
}

} /* namespace opt */
} /* namespace xpp */
