/**
 @file    nlp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/zmp/nlp.h>

namespace xpp {
namespace zmp {

NLP::NLP ()
    :cf_num_diff_functor_(1*std::numeric_limits<double>::epsilon())
{
}

NLP::~NLP ()
{
  // TODO Auto-generated destructor stub
}

void
NLP::Init (OptimizationVariablesPtr& opt_variables,
           const CostContainer& costs,
           const ConstraintContainer& constraints)
{
  opt_variables_ = std::move(opt_variables); // transfer ownership
  constraints_ = constraints;

  costs_ = costs;
  cf_num_diff_functor_.AddCosts(*opt_variables_, costs_);
}

int
NLP::GetNumberOfOptimizationVariables () const
{
  return opt_variables_->GetOptimizationVariableCount();
}

NLP::BoundVec
NLP::GetBoundsOnOptimizationVariables () const
{
  return opt_variables_->GetOptimizationVariableBounds();
}

NLP::VectorXd
NLP::GetStartingValues () const
{
  return opt_variables_->GetOptimizationVariables();
}

double
NLP::EvaluateCostFunction () const
{
  return costs_.EvaluateTotalCost();
}

NLP::VectorXd
NLP::EvaluateCostFunctionGradient () const
{
  Eigen::MatrixXd jacobian(1, GetNumberOfOptimizationVariables());
  cf_num_diff_functor_.df(opt_variables_->GetOptimizationVariables(), jacobian);
  return jacobian;
}

int
NLP::GetNumberOfConstraints () const
{
  return constraints_.GetBounds().size();
}

NLP::BoundVec
NLP::GetBoundsOnConstraints () const
{
  return constraints_.GetBounds();
}

NLP::VectorXd
NLP::EvaluateConstraints () const
{
  return constraints_.EvaluateConstraints();
}

} /* namespace zmp */
} /* namespace xpp */
