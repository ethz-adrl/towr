/**
 @file    com_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/base_motion.h>

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

  base.lin = GetLinear(t_global);

  return base;
}

StateLin3d
BaseMotion::GetLinear (double t_global) const
{
  return linear_->GetPoint(t_global);
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

  // linear part
  for (auto d : {LX, LY, LZ})
    jac.row(d) = linear_->GetJacobian(t_global, dxdt, To3D(d));

  // angular part zero

  return jac;
}

PolynomialSpline
xpp::opt::BaseMotion::GetComSpline () const
{
  return *linear_;
}

} /* namespace opt */
} /* namespace xpp */

