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
  for (auto& c : constraints)
    constraints_.push_back(c);

  RefreshBounds ();
}

ConstraintContainer::VectorXd
ConstraintContainer::GetConstraintValues () const
{
  VectorXd g_all(bounds_.size());

  int c = 0;
  for (const auto& constraint : constraints_) {

    VectorXd g = constraint->GetConstraintValues();
    int c_new = g.rows();
    g_all.middleRows(c, c_new) = g;
    c += c_new;
  }
  return g_all;
}

ConstraintContainer::Jacobian
ConstraintContainer::GetConstraintJacobian () const
{
  int row = 0;
  for (const auto& constraint : constraints_) {

    const Jacobian& jac = constraint->GetConstraintJacobian();
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian_.coeffRef(row+it.row(), it.col()) = it.value();

    row += constraint->GetNumberOfConstraints();
  }
  return jacobian_;
}

void
ConstraintContainer::RefreshBounds ()
{
  bounds_.clear();
  for (const auto& constraint : constraints_) {
    VecBound b = constraint->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }

  int n_constraints = bounds_.size();
  int n_var = constraints_.front()->GetNumberOfOptVariables(); // all the same
  jacobian_ = Jacobian(n_constraints, n_var);
}

VecBound
ConstraintContainer::GetBounds () const
{
  return bounds_;
}

} /* namespace opt */
} /* namespace xpp */
