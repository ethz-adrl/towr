/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/opt/cost_constraint_factory.h>

#include <Eigen/Dense>
#include <map>
#include <stdexcept>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>

#include <xpp/matrix_vector.h>
#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/constraints/foothold_constraint.h>
#include <xpp/opt/constraints/linear_constraint.h>
#include <xpp/opt/constraints/range_of_motion_constraint.h>
#include <xpp/opt/costs/polynomial_cost.h>
#include <xpp/opt/costs/soft_constraint.h>
#include <xpp/opt/polynomial_spline.h>
#include <xpp/opt/variables/variable_names.h>

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
                             const EndeffectorsPos& ee_pos,
                             const State3dEuler& initial_base,
                             const State3dEuler& final_base)
{
  opt_vars_ = opt_vars;
  auto base_lin = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::base_linear));
  base_lin_spline_eq_ = LinearSplineEquations(*base_lin);

  auto base_ang = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::base_angular));
  base_ang_spline_eq_ = LinearSplineEquations(*base_ang);

  params = _params;

  initial_ee_W_ = ee_pos;
  initial_base_ = initial_base;
  final_base_ = final_base;
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
  MatVec lin_eq = base_lin_spline_eq_.MakeStateConstraint(initial_base_.lin,
                                                          t,
                                                          {kPos, kVel, kAcc});
  auto base_linear = std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq, id::base_linear);
  state_constraints->AddComponent(base_linear);


  MatVec ang_eq = base_ang_spline_eq_.MakeStateConstraint(initial_base_.ang,
                                                          t,
                                                          {kPos, kVel, kAcc});

  auto base_angular = std::make_shared<LinearEqualityConstraint>(opt_vars_, ang_eq, id::base_angular);
  state_constraints->AddComponent(base_angular);

  return state_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeFinalConstraint () const
{
  auto state_constraints = std::make_shared<Composite>("State Final Constraints", true);

  MatVec lin_eq_lin = base_lin_spline_eq_.MakeStateConstraint(final_base_.lin,
                                                   params->GetTotalTime(),
                                                   {kPos, kVel, kAcc});

  auto base_linear = std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq_lin, id::base_linear);
  state_constraints->AddComponent(base_linear);

  MatVec lin_eq_ang = base_ang_spline_eq_.MakeStateConstraint(final_base_.ang,
                                                        params->GetTotalTime(),
                                                        {kPos, kVel, kAcc});

  auto base_angular= std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq_ang, id::base_angular);
  state_constraints->AddComponent(base_angular);

  return state_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeJunctionConstraint () const
{
  auto junction_constraints = std::make_shared<Composite>("Junctions Constraints", true);

  auto base_linear = std::make_shared<LinearEqualityConstraint>(
      opt_vars_,
      base_lin_spline_eq_.MakeJunction(),
      id::base_linear);
  junction_constraints->AddComponent(base_linear);

  auto base_angular = std::make_shared<LinearEqualityConstraint>(
      opt_vars_,
      base_ang_spline_eq_.MakeJunction(),
      id::base_angular);
  junction_constraints->AddComponent(base_angular);

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
      opt_vars_, initial_ee_W_, 0.0);

  stance_constraints->AddComponent(constraint_initial);

  // calculate endeffector position in world frame
  Eigen::Matrix3d b_R_w = AngularStateConverter::GetRotationMatrixWorldToBase(final_base_.ang.p_);

  EndeffectorsPos nominal_B = params->GetNominalStanceInBase();
  EndeffectorsPos endeffectors_final_W(nominal_B.GetCount());
  for (auto ee : endeffectors_final_W.GetEEsOrdered())
    endeffectors_final_W.At(ee) = final_base_.lin.p_ + b_R_w.inverse()*nominal_B.At(ee);


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
  auto base_acc_cost = std::make_shared<Composite>("Base Acceleration Costs", false);

  MotionDerivative dxdt = kAcc;
  // careful, must match spline dimension
  VectorXd weight_xyz(3); weight_xyz << 1.0, 1.0, 1.0;


  Eigen::MatrixXd term = base_lin_spline_eq_.MakeCostMatrix(weight_xyz, dxdt);

  MatVec mv(term.rows(), term.cols());
  mv.M = term;
  mv.v.setZero();

  auto base_lin = std::make_shared<QuadraticPolynomialCost>(opt_vars_, mv, id::base_linear,weight);
  base_acc_cost->AddComponent(base_lin);




  VectorXd weight_ypr(3); weight_ypr << 0.1, 0.1, 0.1;
  Eigen::MatrixXd term2 = base_ang_spline_eq_.MakeCostMatrix(weight_ypr, dxdt);

  MatVec mv2(term2.rows(), term2.cols());
  mv2.M = term2;
  mv2.v.setZero();

  auto base_ang = std::make_shared<QuadraticPolynomialCost>(opt_vars_, mv2, id::base_angular,weight);
  base_acc_cost->AddComponent(base_ang);





  return base_acc_cost;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::ToCost (const ConstraintPtr& constraint, double weight) const
{
  return std::make_shared<SoftConstraint>(constraint, weight);
}

} /* namespace opt */
} /* namespace xpp */

