/**
 @file    nlp_structure.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Defines the class functions in NlpStructure
 */

#include <xpp/zmp/nlp_structure.h>

namespace xpp {
namespace zmp {

class VariableSet {
public:
  typedef Eigen::VectorXd VectorXd;
  VariableSet(int n_variables);
  virtual ~VariableSet();

  VectorXd GetVariables() const;
  void SetVariables(const VectorXd& x);
private:
  VectorXd x_;
};

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
///////////////////////////////////////////////////////////////////////////////

NlpStructure::NlpStructure()
{
  n_variables_ = 0;
}

NlpStructure::~NlpStructure ()
{
}

void
NlpStructure::AddVariableSet (std::string name, int n_variables)
{
  variable_sets_.emplace(name, VariableSetPtr(new VariableSet(n_variables)));
  n_variables_ += n_variables;
}

void
NlpStructure::Reset ()
{
  variable_sets_.clear();
  n_variables_ = 0;
}

int
NlpStructure::GetOptimizationVariableCount() const
{
  return n_variables_;
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


} // namespace zmp
} // namespace xpp
