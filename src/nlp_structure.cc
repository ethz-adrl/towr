/**
 @file    nlp_structure.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Defines the class functions in NlpStructure
 */

#include "../include/xpp/opt/nlp_structure.h"

namespace xpp {
namespace opt {

VariableSet::VariableSet (int n_variables, std::string id)
{
  x_ = Eigen::VectorXd::Zero(n_variables);
  bounds_.assign(n_variables, AConstraint::kNoBound_);
  id_ = id;
}

VariableSet::~VariableSet ()
{
}

std::string
VariableSet::GetId () const
{
  return id_;
}

VariableSet::VectorXd
VariableSet::GetVariables () const
{
  return x_;
}

VariableSet::VecBound
VariableSet::GetBounds () const
{
  return bounds_;
}

void
VariableSet::SetVariables (const VectorXd& x)
{
  x_ = x;
}
///////////////////////////////////////////////////////////////////////////////

NlpStructure::NlpStructure()
{
}

NlpStructure::~NlpStructure ()
{
}

void
NlpStructure::AddVariableSet (std::string id, int n_variables)
{
  auto new_set = std::make_shared<VariableSet>(n_variables, id);
  assert(GetSet(id) == nullptr); // make sure doesn't exist yet, otherwise call ClearVariables()
  variable_sets_.push_back(new_set);
}

void
NlpStructure::Reset ()
{
  variable_sets_.clear();
}

int
NlpStructure::GetOptimizationVariableCount() const
{
  int c=0;
  for (const auto& set : variable_sets_)
    c += set->GetVariables().rows();

  return c;
}

NlpStructure::VectorXd
NlpStructure::GetAllOptimizationVariables () const
{
  Eigen::VectorXd x(GetOptimizationVariableCount());
  int c = 0;
  for (const auto& set : variable_sets_) {
    const VectorXd& var = set->GetVariables();
    x.middleRows(c, var.rows()) = var;
    c += var.rows();
  }

  return x;
}

const NlpStructure::VariableSetVector
NlpStructure::GetVariableSets () const
{
  return variable_sets_;
}

NlpStructure::VecBound
NlpStructure::GetAllBounds () const
{
  VecBound bounds_;
  for (const auto& set : variable_sets_) {
    const VecBound& b = set->GetBounds();
    bounds_.insert(std::end(bounds_), std::begin(b), std::end(b));
  }

  return bounds_;
}

void
NlpStructure::SetAllVariables(const VectorXd& x_all)
{
  int c = 0;
  for (const auto& set : variable_sets_) {
    int n_var = set->GetVariables().rows();
    set->SetVariables(x_all.middleRows(c,n_var));
    c += n_var;
  }
}

NlpStructure::VariableSetPtr
NlpStructure::GetSet (std::string id) const
{
  for (const auto& s : variable_sets_)
    if (s->GetId() == id)
     return s;

  return nullptr; // set with this id does not exist
}

void
NlpStructure::SetVariables (std::string id, const VectorXd& values)
{
  GetSet(id)->SetVariables(values);
}

NlpStructure::VectorXd
NlpStructure::GetVariables (std::string id) const
{
  return GetSet(id)->GetVariables();
}

} // namespace zmp
} // namespace xpp


