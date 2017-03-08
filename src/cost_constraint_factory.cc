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
                             const MotionTypePtr& _params, const State2d& initial_state,
                             const State2d& final_state)
{
  com_motion = com;
  motion_structure = ms;
  params = _params;
  initial_geom_state_ = initial_state;
  final_geom_state_ = final_state;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::GetConstraint (ConstraintName name) const
{
  switch (name) {
    case InitCom:     return MakeInitialConstraint();
    case FinalCom:    return MakeFinalConstraint();
    case JunctionCom: return MakeJunctionConstraint();
    case Convexity:   return MakeConvexityConstraint();
    case SuppArea:    return MakeSupportAreaConstraint();
    case Dynamic:     return MakeDynamicConstraint();
    case RomBox:      return MakeRangeOfMotionBoxConstraint();
    case FinalStance: return MakeFinalStanceConstraint();
    case Obstacle:    return MakeObstacleConstraint();
    default: throw std::runtime_error("constraint not defined!");
  }
}

CostConstraintFactory::CostPtr
CostConstraintFactory::GetCost(CostName name) const
{
  switch (name) {
    case ComCostID:          return MakeMotionCost();
    case RangOfMotionCostID: return ToCost(MakeRangeOfMotionBoxConstraint());
    case PolyCenterCostID:   return ToCost(MakePolygonCenterConstraint());
    case FinalComCostID:     return ToCost(MakeFinalConstraint());
    case FinalStanceCostID:  return ToCost(MakeFinalStanceConstraint());
    default: throw std::runtime_error("cost not defined!");
  }
}

VariableSet
CostConstraintFactory::SplineCoeffVariables () const
{
  return VariableSet(com_motion->GetCoeffients(), VariableNames::kSplineCoeff);
}

VariableSet
CostConstraintFactory::ContactVariables (const Vector2d initial_pos,
                                         std::vector<Contact>& contacts) const
{
  contacts.clear();

  // contact locations (x,y) of each step
  utils::StdVecEigen2d footholds_W;
  for (auto ee : motion_structure.GetContactIds()) {
    Eigen::Vector2d nominal_B = params->GetNominalStanceInBase().at(ee);
    footholds_W.push_back(nominal_B + initial_pos); // express in world

    ContactBase c;
    c.ee = ee;
    c.id = contacts.size();
    contacts.push_back(c);
  }

  return VariableSet(utils::ConvertStdToEig(footholds_W), VariableNames::kFootholds);
}

VariableSet
CostConstraintFactory::ConvexityVariables () const
{
  // zmp_ clean this up
  int n_lambdas = motion_structure.GetTotalNumberOfNodeContacts();
  VariableSet::VecBound bounds;
  Eigen::VectorXd lambdas(n_lambdas);
  int k=0;
  for (auto node : motion_structure.GetPhaseStampedVec()) {
    int n_contacts_at_node = node.GetAllContacts().size();
    double avg = 1./n_contacts_at_node;
    double min = avg - avg*params->lambda_deviation_percent_;
    double max = 1-min;
    for (int j=0; j<n_contacts_at_node; ++j) {
      lambdas(k++) = avg;
      bounds.push_back(AConstraint::Bound(min, max));
    }
  }

//  lambdas.fill(0.5); // sort of in the middle for 3 contacts per node
//  return VariableSet(lambdas, VariableNames::kConvexity, AConstraint::Bound(0.0, 1.0));
  return VariableSet(lambdas, VariableNames::kConvexity, bounds);
}

VariableSet
CostConstraintFactory::CopVariables () const
{
  int n_nodes = motion_structure.GetPhaseStampedVec().size();
  Eigen::VectorXd cop(n_nodes*kDim2d);
  cop.setZero();
  return VariableSet(cop, VariableNames::kCenterOfPressure);
}




CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeInitialConstraint () const
{
  LinearSplineEquations eq(*com_motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();

  State2d initial_com_state = initial_geom_state_;
  initial_com_state.p += params->offset_geom_to_com_.topRows<kDim2d>();

  constraint->Init(eq.MakeInitial(initial_com_state), "Initial XY");
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeFinalConstraint () const
{
  LinearSplineEquations eq(*com_motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();

  State2d final_com_state = final_geom_state_;
  final_com_state.p += params->offset_geom_to_com_.topRows<kDim2d>();

  constraint->Init(eq.MakeFinal(final_geom_state_, {kPos, kVel, kAcc}), "Final XY");
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeJunctionConstraint () const
{
  LinearSplineEquations eq(*com_motion);
  auto constraint = std::make_shared<LinearSplineEqualityConstraint>();
  constraint->Init(eq.MakeJunction(), "Junction");
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeDynamicConstraint() const
{
  auto constraint = std::make_shared<DynamicConstraint>();
  constraint->Init(*com_motion, motion_structure);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeSupportAreaConstraint() const
{
  auto constraint = std::make_shared<SupportAreaConstraint>();
  constraint->Init(motion_structure);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeConvexityConstraint() const
{
  auto constraint = std::make_shared<ConvexityConstraint>();
  constraint->Init(motion_structure);
  return constraint;
}


CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  auto constraint = std::make_shared<RangeOfMotionBox>(
      params->GetMaximumDeviationFromNominal(),
      params->GetNominalStanceInBase(),
      params->offset_geom_to_com_.topRows<kDim2d>()
      );

  constraint->Init(*com_motion, motion_structure);
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeFinalStanceConstraint () const
{
  auto constr = std::make_shared<FootholdFinalStanceConstraint>(
      motion_structure,
      final_geom_state_.p,
      params->GetNominalStanceInBase());
  return constr;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeObstacleConstraint () const
{
  auto constraint = std::make_shared<ObstacleLineStrip>();
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakePolygonCenterConstraint () const
{
  auto constraint = std::make_shared<PolygonCenterConstraint>();
  constraint->Init(motion_structure);
  return constraint;
}

CostConstraintFactory::CostPtr
CostConstraintFactory::MakeMotionCost() const
{
  LinearSplineEquations eq(*com_motion);
  Eigen::MatrixXd term;

  utils::MotionDerivative dxdt = utils::kAcc;

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
CostConstraintFactory::ToCost (const ConstraintPtr& constraint) const
{
  return std::make_shared<SoftConstraint>(constraint);
}

} /* namespace opt */
} /* namespace xpp */

