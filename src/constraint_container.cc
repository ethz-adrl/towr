/*
 * constraint_container.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraint_container.h>
#include <iostream>

namespace xpp {
namespace zmp {

ConstraintContainer::ConstraintContainer (OptimizationVariables& subject)
    :IObserver(subject)
{
}

ConstraintContainer::~ConstraintContainer ()
{
  // TODO Auto-generated destructor stub
}

void
ConstraintContainer::Update ()
{
  // optimization variables changed. "Observer" pull functionality implemented
  // in the specific constraints, so here nothing to be done.
}

void
ConstraintContainer::ClearConstraints ()
{
  constraints_.clear();
}

void
ConstraintContainer::AddConstraint (ConstraintPtr constraint)
{
  constraints_.push_back(constraint);
  RefreshBounds ();
}

ConstraintContainer::VectorXd
ConstraintContainer::EvaluateConstraints () const
{
  VectorXd g_all(bounds_.size());

  int c = 0;
  for (const auto& constraint : constraints_) {
    constraint->UpdateVariables(subject_);
    VectorXd g = constraint->EvaluateConstraint();
    int c_new = g.rows();
    g_all.middleRows(c, c_new) = g;
    c += c_new;
  }
  return g_all;
}

ConstraintContainer::Jacobian
ConstraintContainer::GetJacobian () const
{
  int m = bounds_.size();
  int n = subject_->GetOptimizationVariableCount();
  Jacobian jac_all(m, n);
  jac_all.setZero();

  int row = 0;
  for (const auto& constraint : constraints_) {
    constraint->UpdateVariables(subject_);

    int col = 0;
    for (const auto& set : subject_->GetVarSets()) {

      Jacobian jac = constraint->GetJacobianWithRespectTo(set->GetId());

      // insert the derivative in the correct position in the overall jacobian
      for (int k=0; k<jac.outerSize(); ++k)
        for (Jacobian::InnerIterator it(jac,k); it; ++it)
          jac_all.insert(row+it.row(), col+it.col()) = it.value();


      col += set->GetVariables().rows();
    }

    row += constraint->GetNumberOfConstraints();
  }
  return jac_all;
}

void
ConstraintContainer::RefreshBounds ()
{
  bounds_.clear();
  for (const auto& constraint : constraints_) {
    constraint->UpdateVariables(subject_);
    VecBound b = constraint->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }
}

ConstraintContainer::VecBound
ConstraintContainer::GetBounds () const
{
  return bounds_;
}

} /* namespace zmp */
} /* namespace xpp */
