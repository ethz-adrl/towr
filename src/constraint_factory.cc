/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/zmp/constraint_factory.h>

#include <xpp/zmp/linear_spline_equations.h>
#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/joint_angles_constraint.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/zmp/obstacle_constraint.h>


#include <xpp/zmp/range_of_motion_cost.h>
#include <xpp/zmp/a_foothold_cost.h>

#include <xpp/zmp/final_state_equation.h>
#include <xpp/zmp/total_acceleration_equation.h>
#include <xpp/zmp/a_spline_cost.h>
#include <xpp/zmp/com_spline4.h>

namespace xpp {
namespace zmp {

ConstraintFactory::ConstraintFactory ()
{
}

ConstraintFactory::~ConstraintFactory ()
{
  // TODO Auto-generated destructor stub
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateAccConstraint (const Vector2d& init_acc_xy,
                                        const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();

  LinearSplineEquations::State2d init;
  init.a = init_acc_xy;

  constraint->Init(eq.MakeInitial(init));
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateFinalConstraint (const State2d& final_state_xy,
                                          const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeFinal(final_state_xy));
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateJunctionConstraint (const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeJunction());
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateZmpConstraint (const OptimizationVariablesInterpreter& interpreter)
{
  auto constraint = std::make_shared<ZmpConstraint>();
  constraint->Init(interpreter);
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateRangeOfMotionConstraint (
    const OptimizationVariablesInterpreter& interpreter)
{
  auto constraint = std::make_shared<RangeOfMotionConstraint>();
  constraint->Init(interpreter);
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateJointAngleConstraint (
    const OptimizationVariablesInterpreter& interpreter)
{
  auto inv_kin = std::make_shared<xpp::hyq::HyqInverseKinematics>();
  auto constraint = std::make_shared<JointAnglesConstraint>();
  constraint->Init(interpreter, inv_kin);
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateObstacleConstraint ()
{
  auto constraint = std::make_shared<ObstacleConstraint>();
  return constraint;
}

ConstraintFactory::CostPtr
ConstraintFactory::CreateAccelerationCost (const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto cost = std::make_shared<QuadraticSplineCost>();
  cost->Init(eq.MakeAcceleration(1.0,3.0));
  return cost;
}

ConstraintFactory::CostPtr
ConstraintFactory::CreateFinalComCost (const State2d& final_state_xy,
                                       const ComSplinePtr& spline)
{
  LinearSplineEquations eq(spline);
  auto cost = std::make_shared<SquaredSplineCost>();
  cost->Init(eq.MakeFinal(final_state_xy));
  return cost;
}

ConstraintFactory::CostPtr
ConstraintFactory::CreateRangeOfMotionCost (const OptimizationVariablesInterpreter& interpreter)
{
  auto cost = std::make_shared<RangeOfMotionCost>();
  cost->Init(interpreter);
  return cost;
}

ConstraintFactory::CostPtr
ConstraintFactory::CreateFinalStanceCost (
    const Vector2d& goal_xy,
    const SupportPolygonContainer& supp_polygon_container)
{
  auto cost = std::make_shared<FootholdGoalCost>();
  cost->Init(goal_xy, supp_polygon_container);
  return cost;
}

} /* namespace zmp */
} /* namespace xpp */

