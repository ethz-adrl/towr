/**
 @file    constraint_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Defines the ConstraintContainer class.
 */

#include <xpp/opt/constraints/constraint_container.h>

#include <iostream>
#include <Eigen/Sparse>

namespace xpp {
namespace opt {

ConstraintContainer::ConstraintContainer ()
{
}

ConstraintContainer::~ConstraintContainer ()
{
}

void
ConstraintContainer::ClearConstraints ()
{
  constraints_.clear();
}

void
ConstraintContainer::AddConstraint (ConstraitPtrVec constraints)
{
  for (auto& c : constraints) {
    constraints_.push_back(c);
    num_constraints_ += c->GetNumberOfConstraints();
//    opt_vars_ = c->opt_vars_; // assume all constraints use same variables
  }
}

ConstraintContainer::VectorXd
ConstraintContainer::GetConstraintValues () const
{
  VectorXd g_all(GetNumberOfConstraints());

  int row = 0;
  for (const auto& c : constraints_) {

    VectorXd g = c->GetConstraintValues();
    int n_rows = g.rows();
    g_all.middleRows(row, n_rows) = g;
    row += n_rows;
  }
  return g_all;
}

//ConstraintContainer::Jacobian
//ConstraintContainer::GetConstraintJacobian () const
//{
//  int row = 0;
//  for (const auto& constraint : constraints_) {
//
//    const Jacobian& jac = constraint->GetConstraintJacobian();
//    for (int k=0; k<jac.outerSize(); ++k)
//      for (Jacobian::InnerIterator it(jac,k); it; ++it)
//        jacobian_.coeffRef(row+it.row(), it.col()) = it.value();
//
//    row += constraint->GetNumberOfConstraints();
//  }
//  return jacobian_;
//}

//void
//ConstraintContainer::RefreshBounds ()
//{
//  VecBound bounds_;
//  bounds_.clear();
//  for (const auto& constraint : constraints_) {
//    VecBound b = constraint->GetBounds();
//    bounds_.insert(bounds_.end(), b.begin(), b.end());
//  }
//
////  int n_constraints = bounds_.size();
////  int n_var = constraints_.front()->GetNumberOfOptVariables(); // all the same
////  jacobian_ = Jacobian(n_constraints, n_var);
//}

VecBound
ConstraintContainer::GetBounds () const
{
  VecBound bounds_;
  for (const auto& c : constraints_) {
    VecBound b = c->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }

  return bounds_;
}

ConstraintContainer::Jacobian
ConstraintContainer::GetConstraintJacobian () const
{
  int n_var = constraints_.front()->GetConstraintJacobian().cols();
  Jacobian jacobian(num_constraints_, n_var);

  int row = 0;
  for (const auto& c : constraints_) {

    const Jacobian& jac = c->GetConstraintJacobian();
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(row+it.row(), it.col()) = it.value();

    row += c->GetNumberOfConstraints();
  }

  return jacobian;
}

} /* namespace opt */
} /* namespace xpp */
