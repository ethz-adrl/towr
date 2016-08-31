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

ConstraintContainer::JacobianPtr
ConstraintContainer::GetJacobian () const
{
  int row = 0;
  for (const auto& constraint : constraints_) {
    constraint->UpdateVariables(subject_);

    int col = 0;
    for (const auto& set : subject_->GetVarSets()) {

      Jacobian jac = constraint->GetJacobianWithRespectTo(set->GetId());

      std::cout << "GotJac for varset " << set->GetId() << "\n";

      // insert the derivative in the correct position in the overall Jacobian
      for (int k=0; k<jac.outerSize(); ++k)
        for (Jacobian::InnerIterator it(jac,k); it; ++it)
          jacobian_->coeffRef(row+it.row(), col+it.col()) = it.value();


      col += set->GetVariables().rows();
    }

    row += constraint->GetNumberOfConstraints();
  }

  static int counter = 0;
  std::cout << "returning jac for the " << counter << " time." << std::endl;
  std::cout << "nonzero entries: " << jacobian_->nonZeros() << std::endl;
  counter++;

  return jacobian_;
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

  int n_constraints = bounds_.size();
  int n_variables   = subject_->GetOptimizationVariableCount();
  jacobian_ = std::make_shared<Jacobian>(n_constraints, n_variables);
}

ConstraintContainer::VecBound
ConstraintContainer::GetBounds () const
{
  return bounds_;
}

} /* namespace zmp */
} /* namespace xpp */
