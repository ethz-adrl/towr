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
ConstraintContainer::AddConstraint (ConstraintPtr constraint,
                                    const std::string& name)
{
  constraints_.emplace(name, constraint);
  RefreshBounds ();
}

AConstraint&
ConstraintContainer::GetConstraint (const std::string& name)
{
  return *constraints_.at(name);
}

ConstraintContainer::VectorXd
ConstraintContainer::EvaluateConstraints () const
{
  VectorXd g_all(bounds_.size());

  int c = 0;
  for (const auto& constraint : constraints_) {
    constraint.second->UpdateVariables(subject_);
    VectorXd g = constraint.second->EvaluateConstraint();
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
    constraint.second->UpdateVariables(subject_);

    Jacobian jac;
    int n_constraints = 0;
    int col_start = 0;
    for (const auto& set : subject_->GetVarSets()) {

      jac = constraint.second->GetJacobianWithRespectTo(set->GetId());

      if (jac.size() != 0) {// constraint dependent on this variable set
        jac_all.block(row,col_start,jac.rows(),jac.cols()) = jac;
        n_constraints = jac.rows();
      }

      col_start += set->GetVariables().rows();

    }

    row += n_constraints;
  }
  return jac_all;
}

void
ConstraintContainer::RefreshBounds ()
{
  bounds_.clear();
  for (const auto& constraint : constraints_) {
    constraint.second->UpdateVariables(subject_);
    VecBound b = constraint.second->GetBounds();
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
