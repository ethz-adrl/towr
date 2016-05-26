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
  // TODO Auto-generated constructor stub
}

ConstraintContainer::~ConstraintContainer ()
{
  // TODO Auto-generated destructor stub
}

void
ConstraintContainer::AddConstraint (const AConstraint& constraint)
{
  constraints_.push_back(&constraint);

  VecBound b = constraint.GetBounds();
  bounds_.insert(bounds_.end(), b.begin(), b.end());

  g_ = VectorXd(bounds_.size());
}

ConstraintContainer::VectorXd
ConstraintContainer::EvaluateConstraints ()
{
  assert(g_.rows() != 0); // the size of constraint must have already been determined

  int c = 0;
  for (auto constraint : constraints_) {
    VectorXd g = constraint->EvaluateConstraint();
    int c_new = g.rows();
    g_.middleRows(c, c_new) = g;
    c += c_new;
  }
  return g_;
}

ConstraintContainer::VecBound
ConstraintContainer::GetBounds ()
{
  return bounds_;
}


} /* namespace zmp */
} /* namespace xpp */
