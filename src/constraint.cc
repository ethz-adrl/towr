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
  name_ = "GiveMeAName";
}

Constraint::~Constraint ()
{
}

int
Constraint::GetNumberOfConstraints () const
{
  return complete_jacobian_.rows();
}

int
Constraint::GetNumberOfOptVariables () const
{
  return complete_jacobian_.cols();
}

void
Constraint::SetDimensions (const std::vector<OptVarPtr>& vars,
                           int num_constraints)
{
  num_constraints_ = num_constraints;

//  g_ = VectorXd::Zero(num_constraints);
  bounds_ = VecBound(num_constraints);

  int num_vars_ = 0;
  for (auto& v : vars) {
    int n = v->GetOptVarCount();
    Jacobian jac(num_constraints, n); // empty jacobians
    jacobians_.push_back({v->GetId(), jac});
    num_vars_ += n;
  }

  complete_jacobian_ = Jacobian(num_constraints, num_vars_);
}

Constraint::Jacobian
Constraint::GetConstraintJacobian ()
{
  int col = 0;
  for (const auto& v : jacobians_) {
    const Jacobian& jac = v.second;

    // insert the derivative in the correct position in the overall Jacobian
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        complete_jacobian_.coeffRef(it.row(), col+it.col()) = it.value();

    col += jac.cols();
  }

  return complete_jacobian_;
}

Constraint::Jacobian
Constraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empty matrix

  for (const auto& var : jacobians_)
    if (var.first == var_set)
      jac = var.second;

  return jac;
}

// zmp_ DRY with above -.-
Constraint::Jacobian&
Constraint::GetJacobianRefWithRespectTo (std::string var_set)
{
  for (auto& var : jacobians_)
    if (var.first == var_set)
      return var.second;

  assert(false); // Jacobian does not exist
}

VecBound
Constraint::GetBounds ()
{
  UpdateBounds();
  return bounds_;
}

void
Constraint::Update ()
{
  UpdateJacobians();
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

