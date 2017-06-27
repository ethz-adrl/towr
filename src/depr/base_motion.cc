/**
 @file    com_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/base_motion.h>
#include <kindr/Core>

namespace xpp {
namespace opt {

BaseMotion::BaseMotion (const PolySplinePtr& linear,
                        const PolySplinePtr& angular)
  : Composite("base_motion", true)
{
  AddComponent(linear);
  AddComponent(angular);

  // retain pointer to access derived class functions
  linear_  = linear;
  angular_ = angular;
}

BaseMotion::~BaseMotion ()
{
}

State3d
BaseMotion::GetBase (double t_global) const
{
  State3d base; // positions and orientations set to zero

  base.lin = linear_->GetPoint(t_global);

  StateLin3d rpy = angular_->GetPoint(t_global);

  kindr::EulerAnglesXyzD euler(rpy.p_);
  kindr::RotationQuaternionD quat(euler);


  base.ang.q = quat.toImplementation();

  return base;
}

double
BaseMotion::GetTotalTime () const
{
  return linear_->GetTotalTime();
}

JacobianRow
BaseMotion::GetJacobian (double t_global, MotionDerivative dxdt, Coords6D dim) const
{
  return GetJacobian(t_global, dxdt).row(dim);
}

Jacobian
BaseMotion::GetJacobian (double t_global, MotionDerivative dxdt) const
{
  Jacobian jac(kDim6d,GetRows());

//  // angular part zero
//  for (auto d : {AX, AY, AZ})
//    jac.row(d) = angular_->GetJacobian(t_global, dxdt, To3D(d));

  // linear part
  for (auto d : {LX, LY, LZ})
    jac.row(d) = linear_->GetJacobian(t_global, dxdt, To3D(d));

  return jac;
}

PolynomialSpline
xpp::opt::BaseMotion::GetLinearSpline () const
{
  return *linear_;
}

PolynomialSpline
xpp::opt::BaseMotion::GetAngularSpline () const
{
  return *angular_;
}

} /* namespace opt */
} /* namespace xpp */

