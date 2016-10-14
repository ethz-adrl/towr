/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class CostAdapter
 */

#include <xpp/opt/cost_adapter.h>

namespace xpp {
namespace opt {

CostAdapter::CostAdapter ()
{
  constraint_ = nullptr;
}

CostAdapter::CostAdapter (const ConstraintPtr& constraint,
                          const VectorXd& weights)
{
}

CostAdapter::~CostAdapter ()
{
  // TODO Auto-generated destructor stub
}

double
CostAdapter::EvaluateCost () const
{
  double cost = 0.0;
  return cost;
}

void
CostAdapter::UpdateVariables (const OptimizationVariables*)
{
}

CostAdapter::VectorXd
CostAdapter::EvaluateGradientWrt (std::string var_set)
{
  VectorXd grad;
  return grad;
}

} /* namespace opt */
} /* namespace xpp */
