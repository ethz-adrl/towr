/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/opt/cost_constraint_factory.h>

#include <xpp/opt/a_linear_constraint.h>
#include <xpp/opt/a_spline_cost.h>
#include <xpp/opt/linear_spline_equations.h>
#include <xpp/opt/range_of_motion_constraint.h>
#include <xpp/opt/obstacle_constraint.h>
#include <xpp/opt/a_foothold_constraint.h>
#include <xpp/opt/convexity_constraint.h>
#include <xpp/opt/support_area_constraint.h>
#include <xpp/opt/dynamic_constraint.h>
#include <xpp/opt/polygon_center_constraint.h>
#include <xpp/opt/soft_constraint.h>

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

void
CostConstraintFactory::Init (const ComMotionPtr& com, const MotionStructure& ms,
                             const MotionTypePtr& _params)
{
  com_motion = com;
  motion_structure = ms;
  params = _params;
}

VariableSet
CostConstraintFactory::SplineCoeffVariables ()
{
  return VariableSet(com_motion->GetCoeffients(), VariableNames::kSplineCoeff);
}

VariableSet
CostConstraintFactory::ContactVariables (const Vector2d initial_pos)
{
  // contact locations (x,y) of each step
  utils::StdVecEigen2d footholds_W;
  for (auto ee : motion_structure.GetContactIds()) {
    Eigen::Vector2d nominal_B = params->GetNominalStanceInBase().at(ee);
    footholds_W.push_back(nominal_B + initial_pos); // express in world
  }

  return VariableSet(utils::ConvertStdToEig(footholds_W), VariableNames::kFootholds);
}

VariableSet
CostConstraintFactory::ConvexityVariables ()
{
  Eigen::VectorXd lambdas(motion_structure.GetTotalNumberOfNodeContacts());
  lambdas.fill(0.33); // sort of in the middle for 3 contacts per node
  return VariableSet(lambdas, VariableNames::kConvexity, AConstraint::Bound(0.0, 1.0));
}

VariableSet
CostConstraintFactory::CopVariables ()
{
  int n_nodes = motion_structure.GetPhaseStampedVec().size();
  Eigen::VectorXd cop(n_nodes*kDim2d);
  cop.setZero();
  return VariableSet(cop, VariableNames::kCenterOfPressure);
}




CostConstraintFactory::ConstraintPtr
CostConstraintFactory::InitialConstraint_ (const State2d& init)
{
  LinearSplineEquations eq(*com_motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeInitial(init), "Initial XY");
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::FinalConstraint_ (const State2d& final_state_xy)
{
  LinearSplineEquations eq(*com_motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeFinal(final_state_xy, {kPos, kVel}), "Final XY");
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::JunctionConstraint_ ()
{
  LinearSplineEquations eq(*com_motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeJunction(), "Junction");
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::DynamicConstraint_(double robot_height)
{
  auto constraint = std::make_shared<DynamicConstraint>();
  constraint->Init(*com_motion, motion_structure, robot_height);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::SupportAreaConstraint_()
{
  auto constraint = std::make_shared<SupportAreaConstraint>();
  constraint->Init(motion_structure);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::ConvexityConstraint_()
{
  auto constraint = std::make_shared<ConvexityConstraint>();
  constraint->Init(motion_structure);
  return constraint;
}


CostConstraintFactory::ConstraintPtr
CostConstraintFactory::RangeOfMotionBoxConstraint_ ()
{
  auto constraint = std::make_shared<RangeOfMotionBox>(
      params->GetMaximumDeviationFromNominal(),
      params->GetNominalStanceInBase()
      );

  constraint->Init(*com_motion, motion_structure);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::FinalStanceConstraint_ (const Vector2d& goal_xy)
{
  auto constr = std::make_shared<FootholdFinalStanceConstraint>(motion_structure,
                goal_xy, params->GetNominalStanceInBase());
  return constr;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::ObstacleConstraint_ ()
{
  auto constraint = std::make_shared<ObstacleLineStrip>();
  return constraint;
}





CostConstraintFactory::CostPtr
CostConstraintFactory::RangeOfMotionCost_ ()
{
  auto rom_constraint = RangeOfMotionBoxConstraint_();
  auto rom_cost = std::make_shared<SoftConstraint>(rom_constraint);
  return rom_cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::PolygonCenterCost_ ()
{
  auto constraint = std::make_shared<PolygonCenterConstraint>();
  constraint->Init(motion_structure);
  auto cost = std::make_shared<SoftConstraint>(constraint);
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateFinalStanceCost (const Vector2d& goal_xy)
{
  auto final_stance_constraint = FinalStanceConstraint_(goal_xy);
  auto final_stance_cost = std::make_shared<SoftConstraint>(final_stance_constraint);
  return final_stance_cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::ComMotionCost_(const xpp::utils::MotionDerivative dxdt)
{
  LinearSplineEquations eq(*com_motion);
  Eigen::MatrixXd term;

  switch (dxdt) {
    case kAcc:  term = eq.MakeAcceleration(params->weight_com_motion_xy_); break;
    case kJerk: term = eq.MakeJerk(params->weight_com_motion_xy_); break;
    default: assert(false); break; // this cost is not implemented
  }

  xpp::utils::MatVec mv(term.rows(), term.cols());
  mv.M = term;
  mv.v.setZero();

  auto cost = std::make_shared<QuadraticSplineCost>();
  cost->Init(mv);
  return cost;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::CreateFinalComCost (const State2d& final_state_xy)
{
  LinearSplineEquations eq(*com_motion);
  auto cost = std::make_shared<SquaredSplineCost>();
  cost->Init(eq.MakeFinal(final_state_xy, {kPos, kVel, kAcc}));
  return cost;
}


} /* namespace opt */
} /* namespace xpp */

