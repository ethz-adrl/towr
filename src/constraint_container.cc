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

ConstraintContainer::ConstraintContainer ()
{
  bounds_up_to_date_ = true;
}

ConstraintContainer::~ConstraintContainer ()
{
  // TODO Auto-generated destructor stub
}

void
ConstraintContainer::AddConstraint (ConstraintPtr constraint,
                                    const std::string& name)
{
  constraints_.emplace(name, constraint);
  bounds_up_to_date_ = false;
}

ConstraintContainer::VectorXd
ConstraintContainer::EvaluateConstraints () const
{
  assert(bounds_up_to_date_); // call Refresh() if this fails
  VectorXd g_all(bounds_.size());

  int c = 0;
  for (const auto& constraint : constraints_) {
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


} /* namespace zmp */
} /* namespace xpp */
