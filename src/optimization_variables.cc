/**
 @file    optimization_variables.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Defines a class to hold the value of the optimization variables.
 */

#include <xpp/opt/optimization_variables.h>

#include <xpp/opt/nlp_structure.h>

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
//  nlp_structure_.Reset();
  variable_sets_.clear();
}

OptimizationVariables::VectorXd
OptimizationVariables::GetVariables (std::string id) const
{
//  return nlp_structure_.GetVariables(id);
  return GetSet(id).GetVariables();
}

OptimizationVariables::VariableSetVector
OptimizationVariables::GetVarSets () const
{
//  return nlp_structure_.GetVariableSets();
  return variable_sets_;
}

void
OptimizationVariables::AddVariableSet (std::string id, const VectorXd& values, const VarBound& bound)
{
//  nlp_structure_.AddVariableSet(id, values.rows(), bound);
//  nlp_structure_.SetVariables(id, values);


  assert(GetSet(id));// == nullptr); // make sure doesn't exist yet, otherwise call ClearVariables()
//  auto new_set = std::make_shared<VariableSet>(n_variables, id, bound);
  VariableSet new_set(values.rows(), id, bound);
  variable_sets_.push_back(new_set);
  GetSet(id).SetVariables(values);

}

void
OptimizationVariables::SetVariables(const VectorXd& x)
{
//  nlp_structure_.SetAllVariables(x);
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
//  return nlp_structure_.GetAllOptimizationVariables();
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
//  return nlp_structure_.GetAllBounds();
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
//  return nlp_structure_.GetOptimizationVariableCount();
  int n=0;
  for (const auto& set : variable_sets_)
    n += set.GetVariables().rows();

  return n;
}

// private methods
VariableSet&
OptimizationVariables::GetSet (std::string id)
{
  for (auto& set : variable_sets_)
    if (set.GetId() == id)
     return set;

  assert(false);

//  return nullptr; // set with this id does not exist
}

const VariableSet&
OptimizationVariables::GetSet (std::string id) const
{
  for (const auto& set : variable_sets_)
    if (set.GetId() == id)
     return set;

  assert(false);

//  return nullptr; // set with this id does not exist
}


} /* namespace zmp */
} /* namespace xpp */
