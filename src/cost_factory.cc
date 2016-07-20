/**
 @file    cost_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#include <xpp/zmp/cost_factory.h>

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/a_quadratic_cost.h>
#include <xpp/zmp/range_of_motion_cost.h>

#include <xpp/zmp/final_state_equation.h>
#include <xpp/zmp/total_acceleration_equation.h>

namespace xpp {
namespace zmp {

CostFactory::CostFactory ()
{
  // TODO Auto-generated constructor stub
}

CostFactory::~CostFactory ()
{
  // TODO Auto-generated destructor stub
}

CostFactory::CostPtr
CostFactory::CreateAccelerationCost (const ContinuousSplineContainer& splines)
{
  TotalAccelerationEquation eq_total_acc(splines);
  auto cost = std::make_shared<AQuadraticCost>();
  cost->Init(eq_total_acc.BuildLinearEquation());
  return cost;
}

CostFactory::CostPtr
CostFactory::CreateFinalCost (const State2d& final_state_xy,
                              const ContinuousSplineContainer& splines)
{
  FinalStateEquation eq(final_state_xy, splines);
  auto cost = std::make_shared<AQuadraticCost>();
  cost->Init(eq.BuildLinearEquation());
  return cost;
}

CostFactory::CostPtr
CostFactory::CreateRangeOfMotionCost (const OptimizationVariablesInterpreter& interpreter)
{
  auto cost = std::make_shared<RangeOfMotionCost>();
  cost->Init(interpreter);
  return cost;
}


} /* namespace zmp */
} /* namespace xpp */

