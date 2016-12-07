/**
 @file    optimization_variables.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Defines a class to hold the value of the optimization variables.
 */

#include <xpp/opt/optimization_variables.h>
#include <algorithm> // find_if

namespace xpp {
namespace opt {

OptimizationVariables::OptimizationVariables ()
{
}

OptimizationVariables::~OptimizationVariables ()
{
}

void
OptimizationVariables::ClearVariables ()
{
  variable_sets_.clear();
}

OptimizationVariables::VectorXd
OptimizationVariables::GetVariables (std::string id) const
{
  assert(SetExists(id));

  for (const auto& set : variable_sets_)
    if (set.GetId() == id)
      return set.GetVariables();
}

OptimizationVariables::VariableSetVector
OptimizationVariables::GetVarSets () const
{
  return variable_sets_;
}

void
OptimizationVariables::AddVariableSet (const VariableSet& set)
{
  assert(!SetExists(set.GetId())); // ensure that set with this id does not exist yet
  variable_sets_.push_back(set);
}

void
OptimizationVariables::SetAllVariables(const VectorXd& x)
{
  int c = 0;
  for (auto& set : variable_sets_) {
    int n_var = set.GetVariables().rows();
    set.SetVariables(x.middleRows(c,n_var));
    c += n_var;
  }

  NotifyObservers();
}

OptimizationVariables::VectorXd
OptimizationVariables::GetOptimizationVariables () const
{
  Eigen::VectorXd x(GetOptimizationVariableCount());
  int j = 0;
  for (const auto& set : variable_sets_) {
    const VectorXd& var = set.GetVariables();
    x.middleRows(j, var.rows()) = var;
    j += var.rows();
  }

  return x;
}

OptimizationVariables::VecBound
OptimizationVariables::GetOptimizationVariableBounds () const
{
  VecBound bounds_;
  for (const auto& set : variable_sets_) {
    const VecBound& b = set.GetBounds();
    bounds_.insert(std::end(bounds_), std::begin(b), std::end(b));
  }

  return bounds_;
}

int
OptimizationVariables::GetOptimizationVariableCount () const
{
  int n=0;
  for (const auto& set : variable_sets_)
    n += set.GetVariables().rows();

  return n;
}

bool
OptimizationVariables::SetExists (std::string id) const
{
  auto it = std::find_if(variable_sets_.begin(), variable_sets_.end(),
                         [id](const VariableSet& set) { return set.GetId() == id; });
  return it != variable_sets_.end();
}

} /* namespace opt */
} /* namespace xpp */
