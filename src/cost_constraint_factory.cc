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
#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/endeffectors.h>

#include <xpp/matrix_vector.h>
#include <xpp/opt/angular_state_converter.h>
#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/constraints/foothold_constraint.h>
#include <xpp/opt/constraints/linear_constraint.h>
#include <xpp/opt/constraints/range_of_motion_constraint.h>
#include <xpp/opt/costs/polynomial_cost.h>
#include <xpp/opt/costs/soft_constraint.h>
#include <xpp/opt/variables/polynomial_spline.h>
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
CostConstraintFactory::MakePolynomialSplineConstraint (
    const std::string& poly_id, const StateLin3d state, double t) const
{
  auto spline = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars_->GetComponent(poly_id));
  LinearSplineEquations equation_builder(*spline);
  MatVec lin_eq = equation_builder.MakeStateConstraint(state,t, {kPos, kVel, kAcc});
  return std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq, poly_id);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeInitialConstraint () const
{
  auto state_constraints = std::make_shared<Composite>("State Initial Constraints", true);

  double t = 0.0; // initial time
  state_constraints->AddComponent(MakePolynomialSplineConstraint(id::base_linear, initial_base_.lin, t));
  state_constraints->AddComponent(MakePolynomialSplineConstraint(id::base_angular, initial_base_.ang, t));

//  for (auto ee : params->robot_ee_) {
//    std::string id = id::endeffectors_motion+std::to_string(ee);
//    state_constraints->AddComponent(MakePolynomialSplineConstraint(id, StateLin3d(initial_ee_W_.At(ee)), t));
//  }

  return state_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeFinalConstraint () const
{
  auto state_constraints = std::make_shared<Composite>("State Final Constraints", true);

  double T = params->GetTotalTime();

  state_constraints->AddComponent(MakePolynomialSplineConstraint(id::base_linear, final_base_.lin, T));
  state_constraints->AddComponent(MakePolynomialSplineConstraint(id::base_angular, final_base_.ang, T));

  return state_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeJunctionConstraint () const
{
  auto junction_constraints = std::make_shared<Composite>("Junctions Constraints", true);

  junction_constraints->AddComponent(MakePolynomialJunctionConstraint(id::base_linear));
  junction_constraints->AddComponent(MakePolynomialJunctionConstraint(id::base_angular));

//  for (auto ee : params->robot_ee_) {
//    std::string id = id::endeffectors_motion+std::to_string(ee);
//    junction_constraints->AddComponent(MakePolynomialJunctionConstraint(id));
//  }

  return junction_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakePolynomialJunctionConstraint (const std::string& poly_id) const
{
  auto poly = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars_->GetComponent(poly_id));
  LinearSplineEquations equation_builder(*poly);
  return std::make_shared<LinearEqualityConstraint>(opt_vars_, equation_builder.MakeJunction(), poly_id);
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
  Eigen::Matrix3d w_R_b = AngularStateConverter::GetRotationMatrixBaseToWorld(final_base_.ang.p_);

  EndeffectorsPos nominal_B = params->GetNominalStanceInBase();
  EndeffectorsPos endeffectors_final_W(nominal_B.GetCount());
  for (auto ee : endeffectors_final_W.GetEEsOrdered())
    endeffectors_final_W.At(ee) = final_base_.lin.p_ + w_R_b*nominal_B.At(ee);


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

  VectorXd weight_xyz(kDim3d); weight_xyz << 1.0, 1.0, 1.0;
  base_acc_cost->AddComponent(MakePolynomialCost(id::base_linear, weight_xyz, weight));

  VectorXd weight_angular(kDim3d); weight_angular << 0.1, 0.1, 0.1;
  base_acc_cost->AddComponent(MakePolynomialCost(id::base_angular, weight_angular, weight));

  return base_acc_cost;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakePolynomialCost (const std::string& poly_id,
                                           const Vector3d& weight_dimensions,
                                           double weight) const
{
  auto poly = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars_->GetComponent(poly_id));
  LinearSplineEquations equation_builder(*poly);

  Eigen::MatrixXd term = equation_builder.MakeCostMatrix(weight_dimensions, kAcc);

  MatVec mv(term.rows(), term.cols());
  mv.M = term;
  mv.v.setZero();

  return std::make_shared<QuadraticPolynomialCost>(opt_vars_, mv, poly_id, weight);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::ToCost (const ConstraintPtr& constraint, double weight) const
{
  return std::make_shared<SoftConstraint>(constraint, weight);
}

} /* namespace opt */
} /* namespace xpp */

