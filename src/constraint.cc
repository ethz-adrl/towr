/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp/constraint.h>

#include <cassert>
#include <iomanip>
#include <iostream>

namespace xpp {
namespace opt {

Constraint::Constraint ()
{
}

Constraint::~Constraint ()
{
}

int
Constraint::GetNumberOfConstraints () const
{
  return num_constraints_;
}

int
Constraint::GetNumberOfOptVariables () const
{
  return opt_vars_->GetOptimizationVariableCount();
}

void
Constraint::SetDimensions (const OptVarsPtr& vars,
                           int num_constraints)
{
  num_constraints_ = num_constraints;
  opt_vars_ = vars;
}

Constraint::Jacobian
Constraint::GetConstraintJacobian ()
{
  Jacobian jacobian(num_constraints_, GetNumberOfOptVariables());

  int col = 0;
  for (const auto& vars : opt_vars_->GetOptVarsVec()) {

    int n = vars->GetOptVarCount();
    Jacobian jac = Jacobian(num_constraints_, n);

    FillJacobianWithRespectTo(vars->GetId(), jac);

    // insert the derivative in the correct position in the overall Jacobian
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(it.row(), col+it.col()) = it.value();

    col += vars->GetOptVarCount();
  }

  return jacobian;
}

//void
//xpp::opt::Constraint::PrintStatus (double tol) const
//{
//  std::cout << std::setw(17) << std::left << name_;
//  std::cout << "[" << std::setw(3) << std::right << g_.rows() << "]:  ";
//
//  int i=0;
//  for (auto b : bounds_) {
//    bool g_too_small = g_(i) < b.lower_ - tol;
//    bool g_too_large = g_(i) > b.upper_ + tol;
//
//    if (g_too_small || g_too_large)
//      std::cout << i << ",";
//    i++;
//  }
//
//  std::cout << std::endl;
//}


} /* namespace opt */
} /* namespace xpp */

