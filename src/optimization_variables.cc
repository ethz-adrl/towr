/*
 * optimization_variables.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {


//OptimizationVariables&
//OptimizationVariables::GetInstance(int n_spline_coeff, int n_steps)
//{
//  static OptimizationVariables unique_instance(n_spline_coeff, n_steps);
//  return unique_instance;
//}

OptimizationVariables::OptimizationVariables (int n_spline_coeff, int n_steps)
    :nlp_structure_(n_spline_coeff, n_steps)
{
  x_ = VectorXd(nlp_structure_.GetOptimizationVariableCount());
  x_.setZero();
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

OptimizationVariables::StdVecEigen2d
OptimizationVariables::GetFootholds () const
{
  return nlp_structure_.ExtractFootholds(x_);
}

OptimizationVariables::VectorXd
OptimizationVariables::GetSplineCoefficients () const
{
  return nlp_structure_.ExtractSplineCoefficients(x_);
}


} /* namespace zmp */
} /* namespace xpp */


