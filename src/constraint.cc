/*
 * a_constraint.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include "../include/xpp/constraint.h"

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
  // zmp_ DRY with num_constraints
  return GetBounds().size();
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
  auto g = EvaluateConstraint();

  std::cout << std::setw(17) << std::left << name_;
  std::cout << "[" << std::setw(3) << std::right << g.rows() << "]:  ";

  int i=0;
  for (auto b : bounds) {
    bool g_too_small = g(i) < b.lower_ - tol;
    bool g_too_large = g(i) > b.upper_ + tol;

    if (g_too_small || g_too_large)
      std::cout << i << ",";
    i++;
  }

  std::cout << std::endl;
}


} /* namespace opt */
} /* namespace xpp */

