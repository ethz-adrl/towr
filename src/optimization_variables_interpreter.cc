/**
 @file    optimization_variables_interpreter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 3, 2016
 @brief   Brief description
 */

#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

OptimizationVariablesInterpreter::OptimizationVariablesInterpreter ()
{
  // TODO Auto-generated constructor stub
}

OptimizationVariablesInterpreter::~OptimizationVariablesInterpreter ()
{
  // TODO Auto-generated destructor stub
}

void
OptimizationVariablesInterpreter::Init (
    const ComSplinePtr& splines,
    const SupportPolygonContainer& support_polygon_container,
    double robot_height)
{
  spline_structure_ = splines;
  supp_polygon_container_ = support_polygon_container;
  robot_height_ = robot_height;

  initialized_ = true;
}

void
OptimizationVariablesInterpreter::SetFootholds (
    const FootholdPositionsXY& x_feet)
{
  supp_polygon_container_.SetFootholdsXY(x_feet);
}

void
OptimizationVariablesInterpreter::SetSplineCoefficients (
    const VectorXd& x_spline_coeff)
{
  spline_structure_->SetCoefficients(x_spline_coeff);
}

OptimizationVariablesInterpreter::VecFoothold
OptimizationVariablesInterpreter::GetFootholds () const
{
  return supp_polygon_container_.GetFootholds();
}

OptimizationVariablesInterpreter::VecSpline
OptimizationVariablesInterpreter::GetSplines () const
{
  assert(initialized_);
  return spline_structure_->GetPolynomials();
}

double
OptimizationVariablesInterpreter::GetRobotHeight () const
{
  return robot_height_;
}

OptimizationVariablesInterpreter::ComSplinePtr
OptimizationVariablesInterpreter::GetSplineStructure () const
{
  return spline_structure_;
}

OptimizationVariablesInterpreter::VecFoothold
OptimizationVariablesInterpreter::GetStartStance () const
{
  return supp_polygon_container_.GetStartStance();
}

OptimizationVariablesInterpreter::SupportPolygonContainer
OptimizationVariablesInterpreter::GetSuppPolygonContainer () const
{
  return supp_polygon_container_;
}


} /* namespace zmp */
} /* namespace xpp */


