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

void
OptimizationVariables::Init (const VectorXd& x_coeff_abcd, int n_steps)
{
  nlp_structure_.Init(x_coeff_abcd.rows(), n_steps);
  x_ = VectorXd(nlp_structure_.GetOptimizationVariableCount());
  x_.setZero();

  nlp_structure_.SetSplineCoefficients(x_coeff_abcd, x_);

  initialized_ = true;
}

void
xpp::zmp::OptimizationVariables::SetFootholds (const StdVecEigen2d& footholds)
{
  assert(initialized_);
  nlp_structure_.SetFootholds(footholds, x_);
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

} /* namespace zmp */
} /* namespace xpp */

