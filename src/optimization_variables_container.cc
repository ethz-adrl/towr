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
OptimizationVariablesContainer::ClearComponents ()
{
  components_.clear();
}

OptimizationVariablesContainer::OptVarsPtr
OptimizationVariablesContainer::GetComponent (std::string id) const
{
//  assert(SetExists(id));

  for (const auto& set : components_)
    if (set->GetName() == id)
      return set;
}

OptimizationVariablesContainer::OptVarsVec
OptimizationVariablesContainer::GetComponents () const
{
  return components_;
}

void
OptimizationVariablesContainer::AddComponent (const OptVarsPtr& set)
{
//  assert(!SetExists(set->GetName())); // ensure that set with this id does not exist yet
  components_.push_back(set);
}

void
OptimizationVariablesContainer::SetValues(const VectorXd& x)
{
  int c = 0;
  for (auto& set : components_) {
    int n_var = set->GetValues().rows();
    set->SetValues(x.middleRows(c,n_var));
    c += n_var;
  }
}

OptimizationVariablesContainer::VectorXd
OptimizationVariablesContainer::GetValues () const
{
  Eigen::VectorXd x(GetRows());
  int j = 0;
  for (const auto& set : components_) {
    const VectorXd& var = set->GetValues();
    x.middleRows(j, var.rows()) = var;
    j += var.rows();
  }

  return x;
}

VecBound
OptimizationVariablesContainer::GetBounds () const
{
  VecBound bounds_;
  for (const auto& set : components_) {
    const VecBound& b = set->GetBounds();
    bounds_.insert(std::end(bounds_), std::begin(b), std::end(b));
  }

  return bounds_;
}

int
OptimizationVariablesContainer::GetRows () const
{
  int n=0;
  for (const auto& set : components_)
    n += set->GetValues().rows();

  return n;
}

//bool
//OptimizationVariablesContainer::SetExists (std::string id) const
//{
//  auto it = std::find_if(opt_vars_vec_.begin(), opt_vars_vec_.end(),
//                         [id](const OptVarsPtr& set) { return set->GetName() == id; });
//  return it != opt_vars_vec_.end();
//}



} /* namespace opt */
} /* namespace xpp */
