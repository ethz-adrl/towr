/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp/constraint.h>
#include <iostream>
#include <iomanip>

namespace xpp {
namespace opt {

Constraint::Constraint ()
{
  name_ = "GiveMeAName";
}

Constraint::~Constraint ()
{
  // TODO Auto-generated destructor stub
}

int
Constraint::GetNumberOfConstraints () const
{
  return num_constraints_;
}

void
Constraint::SetDependentVariables (const std::vector<ParametrizationPtr>& vars, int num_constraints)
{
  num_constraints_ = num_constraints;
  g_ = VectorXd(num_constraints);
  bounds_ = VecBound(num_constraints);

  for (auto& v : vars) {
    int n = v->GetOptVarCount();
    Jacobian jac(num_constraints, n);
    variables_.push_back({v, jac});
  }
}

void
Constraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  for (auto& var : variables_) {
    VectorXd x = opt_var->GetVariables(var.first->GetID());
    var.first->SetOptimizationParameters(x);
  }

  UpdateJacobians();
  UpdateConstraintValues();
}

Constraint::Jacobian
Constraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  for (const auto& var : variables_)
    if (var.first->GetID() == var_set)
      jac = var.second;

  return jac;
}

Constraint::Jacobian&
Constraint::GetJacobianRefWithRespectTo (std::string var_set)
{
  for (auto& var : variables_)
    if (var.first->GetID() == var_set)
      return var.second;
}


void
xpp::opt::Constraint::PrintStatus (double tol) const
{
  auto bounds = GetBounds();

  std::cout << std::setw(17) << std::left << name_;
  std::cout << "[" << std::setw(3) << std::right << g_.rows() << "]:  ";

  int i=0;
  for (auto b : bounds) {
    bool g_too_small = g_(i) < b.lower_ - tol;
    bool g_too_large = g_(i) > b.upper_ + tol;

    if (g_too_small || g_too_large)
      std::cout << i << ",";
    i++;
  }

  std::cout << std::endl;
}

Constraint::VectorXd
Constraint::GetConstraintValues () const
{
  return g_;
}

} /* namespace opt */
} /* namespace xpp */

