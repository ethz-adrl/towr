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
  int n = floor(T/dt);
  std::cout << "in CoP, n: " << n << std::endl;
  cop_ = VectorXd::Zero(n*kDim2d);
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

} /* namespace opt */
} /* namespace xpp */
