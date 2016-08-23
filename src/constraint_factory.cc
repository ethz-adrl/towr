/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/zmp/constraint_factory.h>

#include <xpp/zmp/initial_acceleration_equation.h>
#include <xpp/zmp/final_state_equation.h>
#include <xpp/zmp/spline_junction_equation.h>
#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/joint_angles_constraint.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/zmp/obstacle_constraint.h>

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
                                        const ComSpline4& splines)
{
  InitialAccelerationEquation eq(init_acc_xy, splines);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.BuildLinearEquation());
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateFinalConstraint (const State2d& final_state_xy,
                                          const ComSpline4& splines)
{
  FinalStateEquation eq(final_state_xy, splines);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.BuildLinearEquation());
  return constraint;
}

ConstraintFactory::ConstraintPtr
ConstraintFactory::CreateJunctionConstraint (const ComSpline4& splines)
{
  SplineJunctionEquation eq(splines);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.BuildLinearEquation());
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

} /* namespace zmp */
} /* namespace xpp */

