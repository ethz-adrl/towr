/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/opt/cost_constraint_factory.h>

#include <xpp/opt/a_linear_constraint.h>
#include <xpp/opt/a_spline_cost.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/linear_spline_equations.h>
#include <xpp/opt/obstacle_constraint.h>
#include <xpp/opt/range_of_motion_constraint.h>
#include <xpp/opt/zmp_constraint.h>
#include <xpp/opt/cost_adapter.h>
#include <xpp/opt/a_foothold_constraint.h>
#include <xpp/opt/convexity_constraint.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/hyq/hyq_robot_interface.h>
#include <xpp/opt/support_area_constraint.h>
#include <xpp/opt/dynamic_constraint.h>
#include <xpp/opt/polygon_center_constraint.h>

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

VariableSet
CostConstraintFactory::CreateSplineCoeffVariables (const ComMotion& com_motion)
{
  return VariableSet(com_motion.GetCoeffients(), VariableNames::kSplineCoeff);
}

VariableSet
CostConstraintFactory::CreateContactVariables (const MotionStructure& motion_structure,
                                               const Vector2d initial_pos)
{
  // contact locations (x,y) of each step
  hyq::HyqRobotInterface hyq;
  utils::StdVecEigen2d footholds_W;
  for (auto leg : motion_structure.GetContactIds())
  {
    Eigen::Vector2d nominal_B = hyq.GetNominalStanceInBase(leg);
    footholds_W.push_back(nominal_B + initial_pos); // express in world
  }

  return VariableSet(utils::ConvertStdToEig(footholds_W), VariableNames::kFootholds);
}

VariableSet
CostConstraintFactory::CreateConvexityVariables (const MotionStructure& motion_structure)
{
  Eigen::VectorXd lambdas(motion_structure.GetTotalNumberOfNodeContacts());
  lambdas.fill(0.33); // sort of in the middle for 3 contacts per node
  return VariableSet(lambdas, VariableNames::kConvexity, AConstraint::Bound(0.0, 1.0));
}

VariableSet
CostConstraintFactory::CreateCopVariables (const MotionStructure& motion_structure)
{
  int n_nodes = motion_structure.GetPhaseStampedVec().size();
  Eigen::VectorXd cop(n_nodes*kDim2d);
  cop.setZero();
  return VariableSet(cop, VariableNames::kCenterOfPressure);
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
  constraint->Init(eq.MakeFinal(final_state_xy, {kPos, kVel}));
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
CostConstraintFactory::CreateDynamicConstraint (const ComMotion& com_motion,
                                                const MotionStructure& motion_structure,
                                                double robot_height)
{
  auto constraint = std::make_shared<DynamicConstraint>();
  constraint->Init(com_motion, motion_structure, robot_height);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateSupportAreaConstraint (const MotionStructure& motion_structure)
{
  auto constraint = std::make_shared<SupportAreaConstraint>();
  constraint->Init(motion_structure);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateConvexityContraint (const MotionStructure& motion_structure)
{
  auto constraint = std::make_shared<ConvexityConstraint>();
  constraint->Init(motion_structure);
  return constraint;
}


CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateRangeOfMotionConstraint (const ComMotion& com_motion,
                                                      const MotionStructure& motion_structure)
{
  auto constraint = std::make_shared<RangeOfMotionBox>();
  auto hyq = std::unique_ptr<ARobotInterface>(new xpp::hyq::HyqRobotInterface());

  constraint->Init(com_motion, motion_structure, std::move(hyq));
  return constraint;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateRangeOfMotionCost (const ComMotion& com_motion,
                                                const MotionStructure& motion_structure)
{
  auto rom_constraint = CreateRangeOfMotionConstraint(com_motion, motion_structure);
  auto rom_cost = std::make_shared<CostAdapter>(rom_constraint);
  return rom_cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreatePolygonCenterCost (const MotionStructure& motion_structure)
{
  auto constraint = std::make_shared<PolygonCenterConstraint>();
  constraint->Init(motion_structure);
  auto cost = std::make_shared<CostAdapter>(constraint);
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateFinalStanceCost (
    const Vector2d& goal_xy,
    const MotionStructure& motion_structure)
{
  auto final_stance_constraint = CreateFinalStanceConstraint(goal_xy, motion_structure);
  auto final_stance_cost = std::make_shared<CostAdapter>(final_stance_constraint);
  return final_stance_cost;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::CreateFinalStanceConstraint (const Vector2d& goal_xy,
                                                    const MotionStructure& motion_structure)
{
  auto hyq = std::unique_ptr<ARobotInterface>(new xpp::hyq::HyqRobotInterface());
  auto final_stance_constraint = std::make_shared<FootholdFinalStanceConstraint>(motion_structure,
                                                                                 goal_xy,
                                                                                 std::move(hyq));
  return final_stance_constraint;
}


CostConstraintFactory::CostPtr
CostConstraintFactory::CreateMotionCost (const ComMotion& motion,
                                         const xpp::utils::MotionDerivative dxdt)
{
  LinearSplineEquations eq(motion);
  auto cost = std::make_shared<QuadraticSplineCost>();

  Eigen::MatrixXd term;

  switch (dxdt) {
    case kAcc:  term = eq.MakeAcceleration(1.0,3.0); break;
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



//CostConstraintFactory::ConstraintPtr
//CostConstraintFactory::CreateObstacleConstraint (const Contacts& contacts)
//{
//  auto constraint = std::make_shared<ObstacleLineStrip>();
//  constraint->Init(contacts);
//  return constraint;
//}


} /* namespace opt */
} /* namespace xpp */

