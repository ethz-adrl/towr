/**
 @file    linear_inverted_pendulum.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/opt/com_motion.h>
#include <xpp/opt/linear_inverted_pendulum.h>

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
LinearInvertedPendulum::SetCurrent (const ComPos& pos, const ComVel& vel,
                                    double height)
{
  pos_ = pos;
  vel_ = vel;
  h_ = height;
}

LinearInvertedPendulum::ComAcc
LinearInvertedPendulum::GetDerivative (const Cop& p) const
{
  Eigen::Array2d k1 = (pos_-p)/h_;
//  Eigen::Array2d k2 = 2*h_/(h_*h_ + (pos_-p).array().square());
//
//  ComAcc acc        = k1*(k2   *vel_.array().square() + kGravity);
//  ComAcc acc_approx = k1*(2./h_*vel_.array().square() + kGravity);
  ComAcc acc_zmp    = k1*(                            + kGravity);

  return acc_zmp;
}

LinearInvertedPendulum::JacobianRow
LinearInvertedPendulum::GetJacobianApproxWrtSplineCoeff (
    const ComMotion& com_motion, double t,
    Coords3D dim, const Cop& p) const
{
  JacobianRow pos     = com_motion.GetJacobian(t, kPos, dim);

//  JacobianRow vel2    = com_motion.GetJacobianVelSquared(t,dim);
//  JacobianRow posvel2 = com_motion.GetJacobianPosVelSquared(t, dim);
//  JacobianRow jac_approx = 1./h_*(kGravity*pos + 2./h_*posvel2 - 2./h_*p(dim)*vel2);

  JacobianRow jac_zmp    = 1./h_*(kGravity*pos);

  return jac_zmp;
}

double
LinearInvertedPendulum::GetJacobianApproxWrtCop (d2::Coords dim) const
{
//  double vel2    = std::pow(vel_(dim),2);
//  double jac_approx = 1./h_*(-kGravity -2./h_*vel2);

  double jac_zmp    = 1./h_*(-kGravity);

  return jac_zmp;
}

} /* namespace opt */
} /* namespace xpp */
