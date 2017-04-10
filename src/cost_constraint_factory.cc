/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/opt/cost_constraint_factory.h>

#include <xpp/opt/a_spline_cost.h>
#include <xpp/soft_constraint.h>
#include <xpp/opt/linear_spline_equations.h>

#include <xpp/opt/constraints/range_of_motion_constraint.h>
#include <xpp/opt/constraints/convexity_constraint.h>
#include <xpp/opt/constraints/support_area_constraint.h>
#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/constraints/polygon_center_constraint.h>
#include <xpp/opt/constraints/contact_load_constraint.h>
#include <xpp/opt/constraints/linear_constraint.h>
#include <xpp/opt/constraints/foothold_constraint.h>

namespace xpp {
namespace opt {

CostConstraintFactory::CostConstraintFactory ()
{
}

CostConstraintFactory::~CostConstraintFactory ()
{
  // TODO Auto-generated destructor stub
}

void
CostConstraintFactory::Init (const OptVarsContainer& opt_vars,
                             const ComMotionPtr& com,
                             const EEMotionPtr& _ee_motion,
                             const ContactSchedulePtr& contact_schedule,
                             const EELoadPtr& _ee_load,
                             const CopPtr& _cop,
                             const MotionTypePtr& _params,
                             const RobotStateCartesian& initial_state,
                             const StateLin2d& final_state)
{
  opt_vars_ = opt_vars;

  com_motion = com;
  ee_motion = _ee_motion;
  contact_schedule_ = contact_schedule;
  ee_load = _ee_load;
  cop = _cop;

  params = _params;
  initial_geom_state_ = initial_state;
  final_geom_state_ = final_state;
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::GetConstraint (ConstraintName name) const
{
  switch (name) {
    case InitCom:     return MakeInitialConstraint();
    case FinalCom:    return MakeFinalConstraint();
    case JunctionCom: return MakeJunctionConstraint();
    case Convexity:   return MakeConvexityConstraint();
    case Dynamic:     return MakeDynamicConstraint();
    case RomBox:      return MakeRangeOfMotionBoxConstraint();
    case Stance: return MakeStancesConstraints();
    case Obstacle:    return MakeObstacleConstraint();
    default: throw std::runtime_error("constraint not defined!");
  }
}

CostConstraintFactory::CostPtr
CostConstraintFactory::GetCost(CostName name) const
{
  switch (name) {
    case ComCostID:          return MakeMotionCost();
    case RangOfMotionCostID: return ToCost(MakeRangeOfMotionBoxConstraint().front());
    case PolyCenterCostID:   return ToCost(MakePolygonCenterConstraint().front());
    case FinalComCostID:     return ToCost(MakeFinalConstraint().front());
    case FinalStanceCostID:  return ToCost(MakeStancesConstraints().front());
    default: throw std::runtime_error("cost not defined!");
  }
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeInitialConstraint () const
{
  LinearSplineEquations eq(com_motion);
  StateLin2d initial_com_state = initial_geom_state_.GetBase().lin.Get2D();
  initial_com_state.p += params->offset_geom_to_com_.topRows<kDim2d>();
  MatVec lin_eq = eq.MakeInitial(initial_com_state);

  auto constraint = std::make_shared<LinearEqualityConstraint>(
      opt_vars_, lin_eq, "Initial XY");
  return {constraint};
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeFinalConstraint () const
{
  LinearSplineEquations eq(com_motion);
  StateLin2d final_com_state = final_geom_state_;
  final_com_state.p += params->offset_geom_to_com_.topRows<kDim2d>();
  MatVec lin_eq = eq.MakeFinal(final_geom_state_, {kPos, kVel, kAcc});

  auto constraint = std::make_shared<LinearEqualityConstraint>(
      opt_vars_, lin_eq, "Final XY");
  return {constraint};
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeJunctionConstraint () const
{
  LinearSplineEquations eq(com_motion);
  auto constraint = std::make_shared<LinearEqualityConstraint>(
      opt_vars_, eq.MakeJunction(), "Junction");
  return {constraint};
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeDynamicConstraint() const
{
  auto constraint = std::make_shared<DynamicConstraint>(opt_vars_,
                                                        ee_motion->GetTotalTime(),
                                                        params->dt_nodes_);
  return {constraint};
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  auto constraint = std::make_shared<RangeOfMotionBox>(
      opt_vars_,
      params->dt_nodes_,
      params->GetMaximumDeviationFromNominal(),
      params->GetNominalStanceInBase(),
      ee_motion->GetTotalTime()
      );

  return {constraint};
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeConvexityConstraint() const
{
  auto cop_constrait = std::make_shared<SupportAreaConstraint>(
      opt_vars_, params->dt_nodes_, ee_motion->GetTotalTime());

  auto convexity = std::make_shared<ConvexityConstraint>(opt_vars_);

  auto contact_load = std::make_shared<ContactLoadConstraint>(opt_vars_);

  return {cop_constrait, convexity, contact_load};
}


CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeStancesConstraints () const
{
  ConstraintPtrVec stance_constraints;

  // calculate initial position in world frame
  auto constraint_initial = std::make_shared<FootholdConstraint>(
      opt_vars_, initial_geom_state_.GetEEPos(), 0.0);

  stance_constraints.push_back(constraint_initial);

  // calculate endeffector position in world frame
  EndeffectorsPos nominal_B = params->GetNominalStanceInBase();
  EndeffectorsPos endeffectors_final_W(nominal_B.GetCount());
  for (auto ee : endeffectors_final_W.GetEEsOrdered())
    endeffectors_final_W.At(ee) = final_geom_state_.Make3D().p + nominal_B.At(ee);

  double t = ee_motion->GetTotalTime();

  auto constraint_final = std::make_shared<FootholdConstraint>(
      opt_vars_, endeffectors_final_W,t);

  stance_constraints.push_back(constraint_final);


  return stance_constraints;
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakeObstacleConstraint () const
{
//  auto constraint = std::make_shared<ObstacleLineStrip>();
//  return constraint;
}

CostConstraintFactory::ConstraintPtrVec
CostConstraintFactory::MakePolygonCenterConstraint () const
{
  return {std::make_shared<PolygonCenterConstraint>(opt_vars_)};
}

CostConstraintFactory::CostPtr
CostConstraintFactory::MakeMotionCost() const
{
  LinearSplineEquations eq(com_motion);
  Eigen::MatrixXd term;

  MotionDerivative dxdt = kAcc;

  switch (dxdt) {
    case kAcc:  term = eq.MakeAcceleration(params->weight_com_motion_xy_); break;
    case kJerk: term = eq.MakeJerk(params->weight_com_motion_xy_); break;
    default: assert(false); break; // this cost is not implemented
  }

  MatVec mv(term.rows(), term.cols());
  mv.M = term;
  mv.v.setZero();

  return std::make_shared<QuadraticSplineCost>(opt_vars_, mv, com_motion);
}

CostConstraintFactory::CostPtr
CostConstraintFactory::ToCost (const ConstraintPtr& constraint) const
{
  return std::make_shared<SoftConstraint>(opt_vars_, constraint);
}

} /* namespace opt */
} /* namespace xpp */

