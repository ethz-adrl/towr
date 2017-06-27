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
  base_lin_spline_eq_ = LinearSplineEquations(base_motion->GetLinearSpline());
  base_ang_spline_eq_ = LinearSplineEquations(base_motion->GetAngularSpline());

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
  auto state_constraints = std::make_shared<Composite>("State Initial Constraints", true);

  double t = 0.0; // initial time
  auto initial_com_state = initial_geom_state_.GetBase().lin;
  MatVec lin_eq = base_lin_spline_eq_.MakeStateConstraint(initial_com_state,
                                                          t,
                                                          {kPos, kVel, kAcc});
  auto base_linear = std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq);
  state_constraints->AddComponent(base_linear);


//  StateLin3d initial_rpy_state;
//  initial_rpy_state.p_ << 0.0, 0.0, 0.0;
//
//  std::cout << "initial state: " << initial_rpy_state << std::endl;
//  MatVec ang_eq = base_ang_spline_eq_.MakeStateConstraint(initial_rpy_state,
//                                                          t,
//                                                          {kPos, kVel, kAcc});
//
//  std::cout << "ang_eq.M: " << ang_eq.M << std::endl;
//  std::cout << "ang_eq.v: " << ang_eq.v << std::endl;
//
//  auto base_angular = std::make_shared<LinearEqualityConstraint>(opt_vars_, ang_eq);
////  state_constraints->AddComponent(base_angular);


  return state_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeFinalConstraint () const
{
  auto state_constraints = std::make_shared<Composite>("State Final Constraints", true);

  MatVec lin_eq_lin = base_lin_spline_eq_.MakeStateConstraint(final_geom_state_,
                                                   params->GetTotalTime(),
                                                   {kPos, kVel, kAcc});

  auto base_linear = std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq_lin);
  state_constraints->AddComponent(base_linear);

//  StateLin3d final_rpy_state;
//  final_rpy_state.p_ << 0.0, 0.3, 0.0; // roll, pitch, yaw
//
//  std::cout << "final ori: " << final_rpy_state.p_ << std::endl;
//
//  MatVec lin_eq_ang = base_ang_spline_eq_.MakeStateConstraint(final_rpy_state,
//                                                        params->GetTotalTime(),
//                                                        {kPos, kVel, kAcc});
//
//  auto base_angular= std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq_ang);
//  state_constraints->AddComponent(base_angular);

  return state_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeJunctionConstraint () const
{
  auto junction_constraints = std::make_shared<Composite>("Junctions Constraints", true);

  auto base_linear = std::make_shared<LinearEqualityConstraint>(
      opt_vars_, base_lin_spline_eq_.MakeJunction());
  junction_constraints->AddComponent(base_linear);

//  auto base_angular = std::make_shared<LinearEqualityConstraint>(
//      opt_vars_, base_ang_spline_eq_.MakeJunction());
//  junction_constraints->AddComponent(base_angular);

  return junction_constraints;
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
  double dt = 0.10;

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
  // zmp_ careful, must match spline dimensions
  VectorXd weight_xyz(3); weight_xyz << 1.0, 1.0, 1.0;
  Eigen::MatrixXd term = base_lin_spline_eq_.MakeCostMatrix(weight_xyz, dxdt);

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

