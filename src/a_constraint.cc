/*
 * a_constraint.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/a_constraint.h>
#include <iostream>
#include <iomanip>

namespace xpp {
namespace opt {

AConstraint::AConstraint ()
{
  name_ = "GiveMeAName";
}

AConstraint::~AConstraint ()
{
  // TODO Auto-generated destructor stub
}

int
AConstraint::GetNumberOfConstraints () const
{
  // zmp_ DRY with num_constraints
  return GetBounds().size();
}

void
AConstraint::SetDependentVariables (const std::vector<ParametrizationPtr>& vars, int num_constraints)
{
//  variables_ = vars;
  num_constraints_ = num_constraints;
  g_ = VectorXd(num_constraints);
  bounds_ = VecBound(num_constraints);


  for (auto& v : vars) {
    int n = v->GetOptVarCount();
    num_variables_ += v->GetOptVarCount();

    Jacobian jac(num_constraints, n);


    variables_.push_back({v, jac});


//    jacobians_.push_back(jac);
  }
}

void
AConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  for (auto& var : variables_) {
    VectorXd x = opt_var->GetVariables(var.first->GetID());
    var.first->SetOptimizationParameters(x);
  }
}

AConstraint::Jacobian
AConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  for (const auto& var : variables_)
    if (var.first->GetID() == var_set)
      jac = var.second;

  return jac;
}


void
xpp::opt::AConstraint::PrintStatus (double tol) const
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

