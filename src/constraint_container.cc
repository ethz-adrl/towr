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
  spline_coeff_ = subject_->GetSplineCoefficients();
  footholds_    = subject_->GetFootholdsStd();
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
  Update();
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
    constraint.second->UpdateVariables(this);
    VectorXd g = constraint.second->EvaluateConstraint();
    int c_new = g.rows();
    g_all.middleRows(c, c_new) = g;
    c += c_new;
  }
  return g_all;
}

void
ConstraintContainer::RefreshBounds ()
{
  bounds_.clear();
  for (const auto& constraint : constraints_) {
    constraint.second->UpdateVariables(this);
    VecBound b = constraint.second->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }
}

ConstraintContainer::VecBound
ConstraintContainer::GetBounds () const
{
  return bounds_;
}

const ConstraintContainer::FootholdsXY&
ConstraintContainer::GetFootholds () const
{
  return footholds_;
}

const ConstraintContainer::VectorXd&
ConstraintContainer::GetSplineCoefficients () const
{
  return spline_coeff_;
}

} /* namespace zmp */
} /* namespace xpp */
