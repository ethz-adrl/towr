/*
 * constraint_container.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/constraint_container.h>

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
}

ConstraintContainer::VectorXd
ConstraintContainer::EvaluateConstraints ()
{
  int c = 0;
  for (auto constraint : constraints_) {
    VectorXd g = constraint->EvaluateConstraint();
    int c_new = g.rows();
    g_.middleCols(c, c_new) = g;
    c += c_new;
  }
  return g_;
}

ConstraintContainer::VecBound
ConstraintContainer::GetBounds ()
{
  VecBound bounds;

  for (auto c : constraints_) {
    VecBound b = c->GetBounds();
    bounds.insert(bounds.begin(), b.begin(), b.end());
  }

  g_ = VectorXd(bounds.size());
  return bounds;
}


} /* namespace zmp */
} /* namespace xpp */
