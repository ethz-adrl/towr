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

CenterOfPressure::CenterOfPressure (double dt, double T)
    : Parametrization("center_of_pressure")
{
  dt_ = dt;
  int idx = GetSegment(T);
  int number_of_segments = idx + 1;
  cop_ = VectorXd::Zero(number_of_segments*kDim2d);
}

CenterOfPressure::~CenterOfPressure ()
{
}

int
CenterOfPressure::GetSegment (double t) const
{
  return floor(t/dt_);
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
CenterOfPressure::SetVariables (const VectorXd& x)
{
  cop_ = x;
}

CenterOfPressure::VectorXd
CenterOfPressure::GetVariables () const
{
  return cop_;
}

CenterOfPressure::JacobianRow
CenterOfPressure::GetJacobianWrtCop (double t, d2::Coords dim) const
{
  JacobianRow jac(GetOptVarCount());
  jac.insert(Index(t,dim)) = 1.0;
  return jac;
}

} /* namespace opt */
} /* namespace xpp */
