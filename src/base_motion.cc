/**
 @file    com_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

BaseMotion::BaseMotion () : Component(-1, "base_motion")
{
}

BaseMotion::~BaseMotion ()
{
}

void
BaseMotion::AddComSpline (const ComSpline& com_spline)
{
  com_spline_ = com_spline;
  SetRows(com_spline.GetXYSplineCoeffients().rows());
}

VectorXd
BaseMotion::GetValues () const
{
  return com_spline_.GetXYSplineCoeffients();
}

void
BaseMotion::SetValues (const VectorXd& x)
{
  com_spline_.SetSplineXYCoefficients(x);
}

State3d
BaseMotion::GetBase (double t_global) const
{
  State3d base; // z and orientation all at zero

  StateLin2d com_xy = GetCom(t_global);

  // since the optimized motion is for the CoM and not the geometric center
  base.lin.p.topRows(kDim2d) = com_xy.p - com_spline_.offset_geom_to_com_.topRows<kDim2d>();
  base.lin.p.z() = com_spline_.z_height_;

  base.lin.v.topRows(kDim2d) = com_xy.v;
  base.lin.a.topRows(kDim2d) = com_xy.a;

  return base;
}

StateLin2d
BaseMotion::GetCom (double t_global) const
{
  return com_spline_.GetCom(t_global);
}

double
BaseMotion::GetTotalTime () const
{
  return com_spline_.GetTotalTime();
}

BaseMotion::JacobianRow
BaseMotion::GetJacobian (double t_global, MotionDerivative dxdt,
                                   Coords3D dim) const
{
  return com_spline_.GetJacobian(t_global, dxdt, dim);
}

} /* namespace zmp */
} /* namespace xpp */

