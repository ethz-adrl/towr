/**
 @file    com_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

BaseMotion::BaseMotion (const ComSplinePtr& com_spline)
  : Composite("base_motion", true)
{
  AddComponent(com_spline);
  com_spline_ = com_spline; // retain pointer to keep specific info
}

BaseMotion::~BaseMotion ()
{
}

State3d
BaseMotion::GetBase (double t_global) const
{
  State3d base; // positions and orientations set to zero

//  StateLin3d com = GetCom(t_global);
////  com.p_ -= offset_geom_to_com_;
  base.lin = GetCom(t_global);

  return base;
}

StateLin3d
BaseMotion::GetCom (double t_global) const
{
  return com_spline_->GetCom(t_global);
}

double
BaseMotion::GetTotalTime () const
{
  return com_spline_->GetTotalTime();
}

BaseMotion::JacobianRow
BaseMotion::GetJacobian (double t_global, MotionDerivative dxdt, Coords6D dim) const
{
  JacobianRow jac(GetRows());

  if (LX==dim || dim==LY || dim==LZ)
    jac = com_spline_->GetJacobian(t_global, dxdt, To3D(dim));

  return jac;
}

ComSpline
xpp::opt::BaseMotion::GetComSpline () const
{
  return *com_spline_;
}

//void
//BaseMotion::SetOffsetGeomToCom (const Vector3d& val)
//{
//  offset_geom_to_com_ = val;
//}

} /* namespace opt */
} /* namespace xpp */

