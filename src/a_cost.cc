/**
@file    a_cost.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 17, 2016
@brief   Defines Cost class
 */

#include <xpp/opt/a_cost.h>

namespace xpp {
namespace opt {

ACost::ACost ()
{
  weight_ = 1.0;
}

ACost::~ACost ()
{
}

double
ACost::EvaluateWeightedCost () const
{
  return weight_ * EvaluateCost();
}

ACost::VectorXd
ACost::EvaluateWeightedGradientWrt (std::string var_set)
{
  return weight_ * EvaluateGradientWrt(var_set);
}

void
ACost::SetWeight (double weight)
{
  weight_ = weight;
}

} // namespace opt
} // namespace xpp
