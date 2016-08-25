/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include "../include/xpp/zmp/cost_constraint_factory.h"

#include <xpp/zmp/linear_spline_equations.h>
#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/joint_angles_constraint.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/zmp/obstacle_constraint.h>

#include <xpp/zmp/range_of_motion_cost.h>
#include <xpp/zmp/a_foothold_cost.h>
#include <xpp/zmp/a_spline_cost.h>

namespace xpp {
namespace zmp {

CostConstraintFactory::CostConstraintFactory ()
{
}

CostConstraintFactory::~CostConstraintFactory ()
{
  // TODO Auto-generated destructor stub
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateAccConstraint (const Vector2d& init_acc_xy,
                                        const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();

  LinearSplineEquations::State2d init;
  init.a = init_acc_xy;

  constraint->Init(eq.MakeInitial(init));
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateFinalConstraint (const State2d& final_state_xy,
                                          const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeFinal(final_state_xy));
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateJunctionConstraint (const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeJunction());
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateZmpConstraint (const OptimizationVariablesInterpreter& interpreter)
{
  auto constraint = std::make_shared<ZmpConstraint>();
  constraint->Init(interpreter);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateRangeOfMotionConstraint (
    const OptimizationVariablesInterpreter& interpreter)
{
  auto constraint = std::make_shared<RangeOfMotionConstraint>();
  constraint->Init(interpreter);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateJointAngleConstraint (
    const OptimizationVariablesInterpreter& interpreter)
{
  auto inv_kin = std::make_shared<xpp::hyq::HyqInverseKinematics>();
  auto constraint = std::make_shared<JointAnglesConstraint>();
  constraint->Init(interpreter, inv_kin);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateObstacleConstraint ()
{
  auto constraint = std::make_shared<ObstacleConstraint>();
  return constraint;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateAccelerationCost (const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto cost = std::make_shared<QuadraticSplineCost>();
  cost->Init(eq.MakeAcceleration(1.0,3.0));
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateFinalComCost (const State2d& final_state_xy,
                                       const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto cost = std::make_shared<SquaredSplineCost>();
  cost->Init(eq.MakeFinal(final_state_xy));
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateRangeOfMotionCost (const OptimizationVariablesInterpreter& interpreter)
{
  auto cost = std::make_shared<RangeOfMotionCost>();
  cost->Init(interpreter);
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateFinalStanceCost (
    const Vector2d& goal_xy,
    const SupportPolygonContainer& supp_polygon_container)
{
  auto cost = std::make_shared<FootholdGoalCost>();
  cost->Init(goal_xy, supp_polygon_container);
  return cost;
}

} /* namespace zmp */
} /* namespace xpp */

