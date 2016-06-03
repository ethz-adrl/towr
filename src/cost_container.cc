/**
 @file    cost_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/cost_container.h>

namespace xpp {
namespace zmp {

CostContainer::CostContainer ()
{
  // TODO Auto-generated constructor stub
}

CostContainer::~CostContainer ()
{
  // TODO Auto-generated destructor stub
}

void
CostContainer::AddCost (CostPtr cost, const std::string& name)
{
  costs_.emplace(name, cost);
}

double
CostContainer::EvaluateTotalCost () const
{
 double total_cost = 0.0;
  for (const auto& cost : costs_)
    total_cost += cost.second->EvaluateCost();

  return total_cost;
}

ACost&
CostContainer::GetCost (const std::string& name)
{
  return *costs_.at(name);
}

} /* namespace zmp */
} /* namespace xpp */
