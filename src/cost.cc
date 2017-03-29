/**
@file    a_cost.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 17, 2016
@brief   Defines Cost class
 */

#include "../include/xpp/cost.h"

namespace xpp {
namespace opt {

Cost::Cost ()
{
  weight_ = 1.0;
}

Cost::~Cost ()
{
}

double
Cost::EvaluateWeightedCost () const
{
  return weight_ * EvaluateCost();
}

Cost::VectorXd
Cost::EvaluateWeightedGradientWrt (std::string var_set)
{
  return weight_ * EvaluateGradientWrt(var_set);
}

void
Cost::SetWeight (double weight)
{
  weight_ = weight;
}

} // namespace opt
} // namespace xpp
