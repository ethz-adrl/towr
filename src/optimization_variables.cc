/**
 @file    optimization_variables.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Defines a class to hold the value of the optimization variables.
 */

#include <xpp/opt/optimization_variables.h>

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
  nlp_structure_.Reset();
}

OptimizationVariables::VectorXd
OptimizationVariables::GetVariables (std::string id) const
{
  return nlp_structure_.GetVariables(id);
}

void
OptimizationVariables::SetVariables (std::string id, const VectorXd& values)
{
  nlp_structure_.SetVariables(id, values);
}

NlpStructure::VariableSetVector
OptimizationVariables::GetVarSets () const
{
  return nlp_structure_.GetVariableSets();
}

void
OptimizationVariables::AddVariableSet (std::string id, const VectorXd& values, const VarBound& bound)
{
  nlp_structure_.AddVariableSet(id, values.rows(), bound);
  SetVariables(id, values);
}

void
OptimizationVariables::SetVariables(const VectorXd& x)
{
  nlp_structure_.SetAllVariables(x);
  NotifyObservers();
}

OptimizationVariables::VectorXd
OptimizationVariables::GetOptimizationVariables () const
{
  return nlp_structure_.GetAllOptimizationVariables();
}

OptimizationVariables::VecBound
OptimizationVariables::GetOptimizationVariableBounds () const
{
  return nlp_structure_.GetAllBounds();
}

int
OptimizationVariables::GetOptimizationVariableCount () const
{
  return nlp_structure_.GetOptimizationVariableCount();
}

} /* namespace zmp */
} /* namespace xpp */
