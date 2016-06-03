/*
 * optimization_variables.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/optimization_variables.h>
#include <xpp/hyq/foothold.h>

namespace xpp {
namespace zmp {

OptimizationVariables::OptimizationVariables ()
{
}

void
OptimizationVariables::Init (const Vector2d& start_cog_p,
                             const Vector2d& start_cog_v,
                             const std::vector<xpp::hyq::LegID>& step_sequence,
                             const SplineTimes& times)
{
  spline_structure_.Init(start_cog_p, start_cog_v ,step_sequence, times);
  step_sequence_ = step_sequence;

  nlp_structure_.Init(spline_structure_.GetTotalFreeCoeff(), step_sequence_.size());
  x_ = VectorXd(nlp_structure_.GetOptimizationVariableCount());
  x_.setZero();

  initialized_ = true;
}

void
xpp::zmp::OptimizationVariables::SetFootholds (const StdVecEigen2d& footholds)
{
  assert(initialized_);
  nlp_structure_.SetFootholds(footholds, x_);
}

void
OptimizationVariables::RegisterObserver(IObserver* o)
{
  bool observer_already_registered = std::find(observers_.begin(), observers_.end(), o) != observers_.end();
  if (!observer_already_registered)
    observers_.push_back(o);
}

void
OptimizationVariables::NotifyObservers () const
{
  for (IObserver* const o : observers_)
    o->Update();
}

int
OptimizationVariables::GetOptimizationVariableCount () const
{
  assert(initialized_);
  return nlp_structure_.GetOptimizationVariableCount();
}

void
xpp::zmp::OptimizationVariables::SetVariables (const VectorXd& x)
{
  assert(initialized_);
  x_ = x;
  NotifyObservers();
}

void
xpp::zmp::OptimizationVariables::SetVariables (const double* x)
{
  assert(initialized_);
  x_ = nlp_structure_.ConvertToEigen(x);
  NotifyObservers();
}

OptimizationVariables::StdVecEigen2d
OptimizationVariables::GetFootholdsStd () const
{
  assert(initialized_);
  return nlp_structure_.ExtractFootholdsToStd(x_);
}

OptimizationVariables::VectorXd
OptimizationVariables::GetFootholdsEig () const
{
  assert(initialized_);
  return nlp_structure_.ExtractFootholdsToEig(x_);
}

OptimizationVariables::VectorXd
OptimizationVariables::GetSplineCoefficients () const
{
  assert(initialized_);
  return nlp_structure_.ExtractSplineCoefficients(x_);
}

OptimizationVariables::VecFoothold
OptimizationVariables::GetFootholds () const
{
  assert(initialized_);
  OptimizationVariables::StdVecEigen2d footholds_xy = GetFootholdsStd();

  VecFoothold opt_footholds(footholds_xy.size());
  xpp::hyq::Foothold::SetXy(footholds_xy, opt_footholds);

  uint i=0;
  for (hyq::Foothold& f : opt_footholds)
    f.leg = step_sequence_.at(i++);

  return opt_footholds;
}

OptimizationVariables::VecSpline
OptimizationVariables::GetSplines ()
{
  assert(initialized_);
  spline_structure_.AddOptimizedCoefficients(GetSplineCoefficients());
  return spline_structure_.GetSplines();
}

} /* namespace zmp */
} /* namespace xpp */

