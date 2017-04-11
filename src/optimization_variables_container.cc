/**
 @file    optimization_variables_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Defines a class to hold the value of the optimization variables.
 */

#include <xpp/optimization_variables_container.h>

#include <algorithm> // find_if
#include <cassert>
#include <iterator>

namespace xpp {
namespace opt {

OptimizationVariablesContainer::OptimizationVariablesContainer ()
{
}

OptimizationVariablesContainer::~OptimizationVariablesContainer ()
{
}

void
OptimizationVariablesContainer::ClearVariables ()
{
  opt_vars_vec_.clear();
}

OptimizationVariablesContainer::OptVarsPtr
OptimizationVariablesContainer::GetSet (std::string id) const
{
  assert(SetExists(id));

  for (const auto& set : opt_vars_vec_)
    if (set->GetId() == id)
      return set;
}

OptimizationVariablesContainer::VectorXd
OptimizationVariablesContainer::GetVariables (std::string id) const
{
  return GetSet(id)->GetVariables();
}

OptimizationVariablesContainer::OptVarsVec
OptimizationVariablesContainer::GetOptVarsVec () const
{
  return opt_vars_vec_;
}

void
OptimizationVariablesContainer::AddVariableSet (const OptVarsPtr& set)
{
  assert(!SetExists(set->GetId())); // ensure that set with this id does not exist yet
  opt_vars_vec_.push_back(set);
}

void
OptimizationVariablesContainer::SetAllVariables(const VectorXd& x)
{
  int c = 0;
  for (auto& set : opt_vars_vec_) {
    int n_var = set->GetVariables().rows();
    set->SetVariables(x.middleRows(c,n_var));
    c += n_var;
  }
}

OptimizationVariablesContainer::VectorXd
OptimizationVariablesContainer::GetOptimizationVariables () const
{
  Eigen::VectorXd x(GetOptimizationVariableCount());
  int j = 0;
  for (const auto& set : opt_vars_vec_) {
    const VectorXd& var = set->GetVariables();
    x.middleRows(j, var.rows()) = var;
    j += var.rows();
  }

  return x;
}

VecBound
OptimizationVariablesContainer::GetOptimizationVariableBounds () const
{
  VecBound bounds_;
  for (const auto& set : opt_vars_vec_) {
    const VecBound& b = set->GetBounds();
    bounds_.insert(std::end(bounds_), std::begin(b), std::end(b));
  }

  return bounds_;
}

int
OptimizationVariablesContainer::GetOptimizationVariableCount () const
{
  int n=0;
  for (const auto& set : opt_vars_vec_)
    n += set->GetVariables().rows();

  return n;
}

bool
OptimizationVariablesContainer::SetExists (std::string id) const
{
  auto it = std::find_if(opt_vars_vec_.begin(), opt_vars_vec_.end(),
                         [id](const OptVarsPtr& set) { return set->GetId() == id; });
  return it != opt_vars_vec_.end();
}



} /* namespace opt */
} /* namespace xpp */
