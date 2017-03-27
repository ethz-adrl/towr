/**
 @file    center_of_pressure.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#include <xpp/opt/center_of_pressure.h>
#include <xpp/cartesian_declarations.h>
#include <iostream>

namespace xpp {
namespace opt {

CenterOfPressure::CenterOfPressure ()
{
  // TODO Auto-generated constructor stub
}

CenterOfPressure::~CenterOfPressure ()
{
  // TODO Auto-generated destructor stub
}

void
CenterOfPressure::Init (double dt, double T)
{
  dt_ = dt;
  int N = GetSegment(T);
  cop_ = VectorXd::Zero(N*kDim2d);
}

CenterOfPressure::Vector2d
CenterOfPressure::GetCop (double t) const
{
  return Vector2d(cop_(Index(t,d2::X)), cop_(Index(t,d2::Y)));
}

int
CenterOfPressure::Index (double t, d2::Coords dimension) const
{
  return kDim2d*GetSegment(t) + dimension;
}

void
CenterOfPressure::SetOptimizationVariables (const VectorXd& x)
{
  cop_ = x;
}

CenterOfPressure::VectorXd
CenterOfPressure::GetOptimizationVariables () const
{
  return cop_;
}

int
CenterOfPressure::GetOptVarCount () const
{
  return GetOptimizationVariables().rows();
}

CenterOfPressure::JacobianRow
CenterOfPressure::GetJacobianWrtCop (double t, d2::Coords dim) const
{
  JacobianRow jac(GetOptVarCount());
  jac.insert(Index(t,dim)) = 1.0;
  return jac;
}

int
CenterOfPressure::GetSegment (double t) const
{
  return round(t/dt_);
}

} /* namespace opt */
} /* namespace xpp */
