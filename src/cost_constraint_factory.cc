/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/opt/cost_constraint_factory.h>

#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/hyq/hyq_robot_interface.h>

#include <xpp/opt/a_foothold_cost.h>
#include <xpp/opt/a_linear_constraint.h>
#include <xpp/opt/a_spline_cost.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/linear_spline_equations.h>
#include <xpp/opt/obstacle_constraint.h>
#include <xpp/opt/range_of_motion_constraint.h>
#include <xpp/opt/zmp_constraint.h>
#include <xpp/opt/cost_adapter.h>

namespace xpp {
namespace opt {

using namespace xpp::utils;

CostConstraintFactory::CostConstraintFactory ()
{
}

CostConstraintFactory::~CostConstraintFactory ()
{
  // TODO Auto-generated destructor stub
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateInitialConstraint (const State2d& init,
                                                const ComMotion& motion)
{
  LinearSplineEquations eq(motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeInitial(init));
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateFinalConstraint (const State2d& final_state_xy,
                                              const ComMotion& motion)
{
  LinearSplineEquations eq(motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeFinal(final_state_xy, {kPos, kVel, kAcc}));
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateJunctionConstraint (const ComMotion& motion)
{
  LinearSplineEquations eq(motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeJunction());
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateZmpConstraint (const MotionStructure& motion_structure,
                                            const ComMotion& com_motion,
                                            const Contacts& contacts,
                                            double walking_height)
{
  auto constraint = std::make_shared<ZmpConstraint>();
  constraint->Init(motion_structure, com_motion, contacts, walking_height);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateRangeOfMotionConstraint (const ComMotion& com_motion,
                                                      const Contacts& contacts,
                                                      const MotionStructure& motion_structure)
{
  auto constraint = std::make_shared<RangeOfMotionBox>();
  auto hyq = std::unique_ptr<ARobotInterface>(new xpp::hyq::HyqRobotInterface());

  constraint->Init(com_motion, contacts, motion_structure, std::move(hyq));
  return constraint;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateRangeOfMotionCost (const ComMotion& com_motion,
                                                const Contacts& contacts,
                                                const MotionStructure& motion_structure)
{
  auto rom_constraint = CreateRangeOfMotionConstraint(com_motion, contacts, motion_structure);
  auto rom_cost = std::make_shared<CostAdapter>(rom_constraint);
  return rom_cost;
}

//CostConstraintFactory::ConstraintPtr
//CostConstraintFactory::CreateJointAngleConstraint (
//    const OptimizationVariablesInterpreter& interpreter)
//{
//  auto inv_kine = std::make_shared<xpp::hyq::HyqInverseKinematics>();
//  auto constraint = std::make_shared<JointAnglesConstraint>();
//  constraint->Init(interpreter, inv_kine);
//  return constraint;
//}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateObstacleConstraint (const Contacts& contacts)
{
  auto constraint = std::make_shared<ObstacleLineStrip>();
  constraint->Init(contacts);
  return constraint;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateMotionCost (const ComMotion& motion,
                                         const xpp::utils::MotionDerivative dxdt)
{
  LinearSplineEquations eq(motion);
  auto cost = std::make_shared<QuadraticSplineCost>();

  Eigen::MatrixXd term;

  switch (dxdt) {
    case kAcc:  term = eq.MakeAcceleration(1.0,2.0); break;
    case kJerk: term = eq.MakeJerk(1.0,2.0); break;
    default: assert(false); break; // this cost is not implemented
  }

  xpp::utils::MatVec mv(term.rows(), term.cols());
  mv.M = term;
  mv.v.setZero();

  cost->Init(mv);
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateFinalComCost (const State2d& final_state_xy,
                                       const ComMotion& motion)
{
  LinearSplineEquations eq(motion);
  auto cost = std::make_shared<SquaredSplineCost>();
  cost->Init(eq.MakeFinal(final_state_xy, {kPos, kVel, kAcc}));
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateFinalStanceCost (
    const Vector2d& goal_xy,
    const Contacts& contacts)
{
  auto cost = std::make_shared<FootholdGoalCost>();
  cost->Init(goal_xy, contacts);
  return cost;
}

} /* namespace zmp */
} /* namespace xpp */

