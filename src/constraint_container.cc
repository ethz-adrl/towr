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
ConstraintContainer::AddConstraint (AConstraint& constraint,
                                    const std::string& name)
{
  constraints_.insert ( std::pair<std::string, AConstraint*>(name,&constraint) );
  bounds_up_to_date_ = false;
}

ConstraintContainer::VectorXd
ConstraintContainer::EvaluateConstraints ()
{
  assert(bounds_up_to_date_); // call Refresh() if this fails

  int c = 0;
  for (auto constraint : constraints_) {
    VectorXd g = constraint.second->EvaluateConstraint();
    int c_new = g.rows();
    g_.middleRows(c, c_new) = g;
    c += c_new;
  }
  return g_;
}

AConstraint&
ConstraintContainer::GetConstraint (const std::string& name) const
{
  return *constraints_.find(name)->second;
}

void
ConstraintContainer::Refresh ()
{
  bounds_.clear();
  for (auto constraint : constraints_) {
    VecBound b = constraint.second->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }

  g_ = VectorXd(bounds_.size());
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
