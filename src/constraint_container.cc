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
  bounds_up_to_date_ = true;
}

ConstraintContainer::~ConstraintContainer ()
{
  // TODO Auto-generated destructor stub
}

void
ConstraintContainer::Update ()
{
  spline_coeff_ = subject_->GetSplineCoefficients();
  footholds_ = subject_->GetFootholdsStd();
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
  // smell just update bounds here directly
  bounds_up_to_date_ = false;
}

ConstraintContainer::VectorXd
ConstraintContainer::EvaluateConstraints () const
{
  assert(bounds_up_to_date_); // call Refresh() if this fails
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

AConstraint&
ConstraintContainer::GetConstraint (const std::string& name)
{
  return *constraints_.at(name);
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

  bounds_up_to_date_ = true;
}

ConstraintContainer::VecBound
ConstraintContainer::GetBounds () const
{
  assert(bounds_up_to_date_); // call Refresh() if this fails
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
