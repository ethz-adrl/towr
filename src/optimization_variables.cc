/*
 * optimization_variables.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

OptimizationVariables::OptimizationVariables ()
{
}

OptimizationVariables::~OptimizationVariables ()
{
}

void
OptimizationVariables::Init (int n_spline_coeff, int n_steps)
{
  nlp_structure_.Init(n_spline_coeff, n_steps);
  initialized_ = true;
}

void
OptimizationVariables::Init (const VectorXd& x_coeff_abcd,
                             const StdVecEigen2d& footholds)
{
  Init(x_coeff_abcd.rows(), footholds.size());

  nlp_structure_.SetSplineCoefficients(x_coeff_abcd);
  nlp_structure_.SetFootholds(footholds);
}

void
xpp::zmp::OptimizationVariables::SetVariables (const VectorXd& x)
{
  assert(initialized_);
  nlp_structure_.SetAllVariables(x);
  NotifyObservers();
}

void
xpp::zmp::OptimizationVariables::SetVariables (const double* x)
{
  assert(initialized_);
  nlp_structure_.SetAllVariables(x);
  NotifyObservers();
}

OptimizationVariables::VectorXd
OptimizationVariables::GetOptimizationVariables () const
{
  return nlp_structure_.GetOptimizationVariables();
}

int
OptimizationVariables::GetOptimizationVariableCount () const
{
  assert(initialized_);
  return nlp_structure_.GetOptimizationVariableCount();
}

OptimizationVariables::StdVecEigen2d
OptimizationVariables::GetFootholdsStd () const
{
  assert(initialized_);
  return nlp_structure_.GetFootholdsStd();
}

OptimizationVariables::VectorXd
OptimizationVariables::GetSplineCoefficients () const
{
  assert(initialized_);
  return nlp_structure_.GetSplineCoefficients();
}

} /* namespace zmp */
} /* namespace xpp */
