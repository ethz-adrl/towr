/**
 @file    nlp_structure.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Defines the class functions in NlpStructure
 */

#include <xpp/zmp/nlp_structure.h>

namespace xpp {
namespace zmp {

static constexpr int kDim2d = 2; // X,Y

NlpStructure::NlpStructure(int n_spline_coeff, int n_steps)
{
  Init(n_spline_coeff, n_steps);
}

NlpStructure::~NlpStructure ()
{
}

void
NlpStructure::Init(int n_spline_coeff, int n_steps)
{
  variable_sets_.emplace("spline_coeff", VariableSetPtr(new VariableSet(n_spline_coeff)));
  variable_sets_.emplace("footholds",    VariableSetPtr(new VariableSet(kDim2d*n_steps)));
}

int
NlpStructure::GetOptimizationVariableCount() const
{
  int c = 0;
  for (const auto& set : variable_sets_)
    c += set.second->GetVariables().rows();
  return c;
}

NlpStructure::VectorXd
NlpStructure::GetOptimizationVariables () const
{
  Eigen::VectorXd x(GetOptimizationVariableCount());
  int c = 0;
  for (const auto& set : variable_sets_) {
    const VectorXd& var = set.second->GetVariables();
    x.middleRows(c, var.rows()) = var;
    c += var.rows();
  }

  return x;
}

void
NlpStructure::SetAllVariables(const VectorXd& x_all)
{
  int c = 0;
  for (const auto& set : variable_sets_) {
    int n_var = set.second->GetVariables().rows();
    set.second->SetVariables(x_all.middleRows(c,n_var));
    c += n_var;
  }
}

void
NlpStructure::SetAllVariables(const Number* x_all)
{
  SetAllVariables(ConvertToEigen(x_all));
}

void
NlpStructure::SetVariables (std::string set_name, const VectorXd& values)
{
  variable_sets_.at(set_name)->SetVariables(values);
}

NlpStructure::VectorXd
NlpStructure::GetVariables (std::string set_name) const
{
  return variable_sets_.at(set_name)->GetVariables();
}

NlpStructure::VectorXd
NlpStructure::ConvertToEigen(const Number* x) const
{
  return Eigen::Map<const VectorXd>(x,GetOptimizationVariableCount());
}


// The Variable Set
VariableSet::VariableSet (int n_variables)
{
  x_ = Eigen::VectorXd::Zero(n_variables);
}

VariableSet::~VariableSet ()
{
}

VariableSet::VectorXd
VariableSet::GetVariables () const
{
  return x_;
}

void
VariableSet::SetVariables (const VectorXd& x)
{
  x_ = x;
}

} // namespace zmp
} // namespace xpp
