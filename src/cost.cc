/**
@file    a_cost.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 17, 2016
@brief   Defines Cost class
 */

#include <xpp/cost.h>

namespace xpp {
namespace opt {

Cost::Cost ()
{
  weight_ = 1.0;
  num_constraints_ = 1;
}

Cost::~Cost ()
{
}

Cost::VectorXd
Cost::GetConstraintValues () const
{
  VectorXd cost(1);
  cost(0) = weight_ * GetCost();
  return cost;
}

Cost::Jacobian
Cost::GetConstraintJacobian () const
{
  return weight_ * GetJacobian();
}

void
Cost::SetWeight (double weight)
{
  weight_ = weight;
}

} // namespace opt
} // namespace xpp

