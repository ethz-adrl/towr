/*
 * optimization_variables.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

OptimizationVariables::OptimizationVariables (int n_spline_coeff, int n_steps)
    :nlp_structure_(n_spline_coeff, n_steps)
{
  x_ = VectorXd(nlp_structure_.GetOptimizationVariableCount());
  x_.setZero();
}

void
xpp::zmp::OptimizationVariables::SetFootholds (const StdVecEigen2d& footholds)
{
  nlp_structure_.SetFootholds(footholds, x_);
}

void
OptimizationVariables::RegisterObserver(IObserver* o)
{
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
  return nlp_structure_.GetOptimizationVariableCount();
}

void
xpp::zmp::OptimizationVariables::SetVariables (const VectorXd& x)
{
  x_ = x;
  NotifyObservers();
}

void
xpp::zmp::OptimizationVariables::SetVariables (const double* x)
{
  x_ = nlp_structure_.ConvertToEigen(x);
  NotifyObservers();
}

OptimizationVariables::StdVecEigen2d
OptimizationVariables::GetFootholdsStd () const
{
  return nlp_structure_.ExtractFootholdsToStd(x_);
}

OptimizationVariables::VectorXd
OptimizationVariables::GetFootholdsEig () const
{
  return nlp_structure_.ExtractFootholdsToEig(x_);
}

OptimizationVariables::VectorXd
OptimizationVariables::GetSplineCoefficients () const
{
  return nlp_structure_.ExtractSplineCoefficients(x_);
}


} /* namespace zmp */
} /* namespace xpp */

