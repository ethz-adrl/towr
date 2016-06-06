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
    const Vector2d& start_cog_p, const Vector2d& start_cog_v,
    const std::vector<xpp::hyq::LegID>& step_sequence, const SplineTimes& times)
{
  spline_structure_.Init(start_cog_p, start_cog_v ,step_sequence, times);
  step_sequence_ = step_sequence;

  initialized_ = true;
}

OptimizationVariablesInterpreter::VecFoothold
OptimizationVariablesInterpreter::GetFootholds (const FootholdPositionsXY& footholds_xy) const
{
  assert(initialized_);

  VecFoothold opt_footholds(footholds_xy.size());
  xpp::hyq::Foothold::SetXy(footholds_xy, opt_footholds);

  uint i=0;
  for (hyq::Foothold& f : opt_footholds) {
    f.leg = step_sequence_.at(i++);
    f.p.z() = 0.0;
  }

  return opt_footholds;
}

OptimizationVariablesInterpreter::VecSpline
OptimizationVariablesInterpreter::GetSplines (const VectorXd& spline_coeff_abcd) const
{
  assert(initialized_);
  return spline_structure_.BuildOptimizedSplines(spline_coeff_abcd);
}

} /* namespace zmp */
} /* namespace xpp */
