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

//void
//OptimizationVariablesInterpreter::Init (
//    const ContinuousSplineContainer& splines,
//    const std::vector<xpp::hyq::LegID>& step_sequence,
//    const VecFoothold& start_stance,
//    double robot_height)
//{
//  spline_structure_ = splines;
//  step_sequence_ = step_sequence;
//  start_stance_ = start_stance;
//  robot_height_ = robot_height;
//
//  initialized_ = true;
//}

OptimizationVariablesInterpreter::VecFoothold
OptimizationVariablesInterpreter::GetFootholds (const FootholdPositionsXY& footholds_xy) const
{
  assert(initialized_);

  VecFoothold opt_footholds(footholds_xy.size());
  xpp::hyq::Foothold::SetXy(footholds_xy, opt_footholds);

  uint i=0;
  for (hyq::Foothold& f : opt_footholds) {
    f.leg = supp_polygon_container_.GetLegID(i++);
    f.p.z() = 0.0;
  }

  return opt_footholds;
}

OptimizationVariablesInterpreter::VecSpline
OptimizationVariablesInterpreter::GetSplines (const VectorXd& spline_coeff_abcd) const
{
  assert(initialized_);
  return spline_structure_->BuildOptimizedSplines(spline_coeff_abcd);
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

