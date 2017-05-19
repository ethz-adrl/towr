/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/opt/cost_constraint_factory.h>

#include <array>
#include <cassert>
#include <map>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>

#include <xpp/matrix_vector.h>
#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/constraints/foothold_constraint.h>
#include <xpp/opt/constraints/linear_constraint.h>
//#include <xpp/opt/constraints/polygon_center_constraint.h>
#include <xpp/opt/constraints/range_of_motion_constraint.h>
#include <xpp/opt/costs/polynomial_cost.h>
#include <xpp/opt/costs/soft_constraint.h>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

CostConstraintFactory::CostConstraintFactory ()
{
}

CostConstraintFactory::~CostConstraintFactory ()
{
}

void
CostConstraintFactory::Init (const OptVarsContainer& opt_vars,
                             const MotionParamsPtr& _params,
                             const RobotStateCartesian& initial_state,
                             const StateLin3d& final_state)
{
  opt_vars_ = opt_vars;
  auto base_motion = std::dynamic_pointer_cast<BaseMotion>(opt_vars->GetComponent("base_motion"));
  spline_eq_ = LinearSplineEquations(base_motion->GetComSpline());

  params = _params;
  initial_geom_state_ = initial_state;
  final_geom_state_   = final_state;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::GetConstraint (ConstraintName name) const
{
  switch (name) {
    case InitCom:     return MakeInitialConstraint();
    case FinalCom:    return MakeFinalConstraint();
    case JunctionCom: return MakeJunctionConstraint();
    case Dynamic:     return MakeDynamicConstraint();
    case RomBox:      return MakeRangeOfMotionBoxConstraint();
    case Stance:      return MakeStancesConstraints();
    default: throw std::runtime_error("constraint not defined!");
  }
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::GetCost(CostName name) const
{
  double weight = params->GetCostWeights().at(name);

  switch (name) {
    case ComCostID:          return MakeMotionCost(weight);
    case RangOfMotionCostID: return ToCost(MakeRangeOfMotionBoxConstraint(), weight);
//    case PolyCenterCostID:   return ToCost(MakePolygonCenterConstraint()   , weight);
    case FinalComCostID:     return ToCost(MakeFinalConstraint()           , weight);
    case FinalStanceCostID:  return ToCost(MakeStancesConstraints()        , weight);
    default: throw std::runtime_error("cost not defined!");
  }
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeInitialConstraint () const
{
  auto initial_com_state = initial_geom_state_.GetBase().lin;
//  initial_com_state.p_ += params->offset_geom_to_com_;
  double t = 0.0; // initial time
  MatVec lin_eq = spline_eq_.MakeStateConstraint(initial_com_state,
                                                 t,
                                                 {kPos, kVel, kAcc});

  return std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeFinalConstraint () const
{
  auto final_com_state = final_geom_state_;
//  final_com_state.p_ += params->offset_geom_to_com_;
  MatVec lin_eq = spline_eq_.MakeStateConstraint(final_geom_state_,
                                                 params->GetTotalTime(),
                                                 {kPos, kVel, kAcc});

  return std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeJunctionConstraint () const
{
  auto constraint = std::make_shared<LinearEqualityConstraint>(
      opt_vars_, spline_eq_.MakeJunction());
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeDynamicConstraint() const
{
  double dt = params->duration_polynomial_/params->n_constraints_per_poly_;
  auto constraint = std::make_shared<DynamicConstraint>(opt_vars_,
                                                        params->GetTotalTime(),
                                                        dt
                                                        );
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  double dt = 0.1;

  auto constraint = std::make_shared<RangeOfMotionBox>(
      opt_vars_,
      dt,
      params->GetMaximumDeviationFromNominal(),
      params->GetNominalStanceInBase(),
      params->GetTotalTime()
      );

  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeStancesConstraints () const
{
  auto stance_constraints = std::make_shared<Composite>("Stance Constraints", true);

  // calculate initial position in world frame
  auto constraint_initial = std::make_shared<FootholdConstraint>(
      opt_vars_, initial_geom_state_.GetEEPos(), 0.0);

  stance_constraints->AddComponent(constraint_initial);

  // calculate endeffector position in world frame
  EndeffectorsPos nominal_B = params->GetNominalStanceInBase();
  EndeffectorsPos endeffectors_final_W(nominal_B.GetCount());
  for (auto ee : endeffectors_final_W.GetEEsOrdered())
    endeffectors_final_W.At(ee) = final_geom_state_.p_ + nominal_B.At(ee);


  auto constraint_final = std::make_shared<FootholdConstraint>(
      opt_vars_, endeffectors_final_W, params->GetTotalTime());

  stance_constraints->AddComponent(constraint_final);


  return stance_constraints;
}

//CostConstraintFactory::ConstraintPtr
//CostConstraintFactory::MakePolygonCenterConstraint () const
//{
//  return std::make_shared<PolygonCenterConstraint>(opt_vars_);
//}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeMotionCost(double weight) const
{
  MotionDerivative dxdt = kAcc;
  std::array<double,3> weight_xy = {1.0, 1.0, 1.0};
  Eigen::MatrixXd term = spline_eq_.MakeCostMatrix(weight_xy, dxdt);

  MatVec mv(term.rows(), term.cols());
  mv.M = term;
  mv.v.setZero();

  return std::make_shared<QuadraticPolynomialCost>(opt_vars_, mv, weight);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::ToCost (const ConstraintPtr& constraint, double weight) const
{
  return std::make_shared<SoftConstraint>(constraint, weight);
}

} /* namespace opt */
} /* namespace xpp */

