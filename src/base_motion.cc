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
  // still don't know how many optimization variables
}

void
BaseMotion::SetVariableCount ()
{
  SetRows(GetValues().rows());
}

BaseMotion::~BaseMotion ()
{
}

VectorXd
BaseMotion::GetValues () const
{
  return GetXYSplineCoeffients();
}

void
BaseMotion::SetValues (const VectorXd& x)
{
  SetSplineXYCoefficients(x);
}

VecScalar
BaseMotion::GetLinearApproxWrtCoeff (double t_global, MotionDerivative dxdt, Coords3D dim) const
{
  VecScalar linear_approx; // at current coefficient values

  linear_approx.v = GetJacobian(t_global, dxdt, dim);
  linear_approx.s = GetCom(t_global).GetByIndex(dxdt, dim);

  return linear_approx;
}

void
BaseMotion::SetOffsetGeomToCom (const Vector3d& offset)
{
  offset_geom_to_com_ = offset;
}

State3d
BaseMotion::GetBase (double t_global) const
{
  State3d base; // z and orientation all at zero

  StateLin2d com_xy = GetCom(t_global);

  // since the optimized motion is for the CoM and not the geometric center
  base.lin.p.topRows(kDim2d) = com_xy.p - offset_geom_to_com_.topRows<kDim2d>();
  base.lin.p.z() = z_height_;

  base.lin.v.topRows(kDim2d) = com_xy.v;
  base.lin.a.topRows(kDim2d) = com_xy.a;

  return base;
}

} /* namespace zmp */
} /* namespace xpp */


