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

#include <xpp/opt/constraints/dynamic_constraint.h>
#include <xpp/opt/constraints/linear_constraint.h>
#include <xpp/opt/constraints/range_of_motion_constraint.h>
#include <xpp/opt/costs/polynomial_cost.h>
#include <xpp/opt/costs/soft_constraint.h>

#include <xpp/opt/variables/polynomial_spline.h>
#include <xpp/opt/angular_state_converter.h>
#include <xpp/opt/variables/variable_names.h>

#include <xpp/opt/centroidal_model.h>
#include <xpp/opt/constraints/spline_constraint.h>

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
//  auto spline = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars_->GetComponent(poly_id));
//  LinearSplineEquations equation_builder(*spline);
//  MatVec lin_eq = equation_builder.MakeStateConstraint(state,t, {kPos, kVel, kAcc});
//  return std::make_shared<LinearEqualityConstraint>(opt_vars_, lin_eq, poly_id);

  auto derivs = {kPos, kVel, kAcc};
  return std::make_shared<SplineStateConstraint>(opt_vars_, poly_id, t, state, derivs);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeInitialConstraint () const
{
  auto state_constraints = std::make_shared<Composite>("State Initial Constraints", true);

  double t = 0.0; // initial time
  state_constraints->AddComponent(MakePolynomialSplineConstraint(id::base_linear, initial_base_.lin, t));
  state_constraints->AddComponent(MakePolynomialSplineConstraint(id::base_angular, initial_base_.ang, t));

  for (auto ee : params->robot_ee_) {
    std::string id = id::endeffectors_motion+std::to_string(ee);
    state_constraints->AddComponent(MakePolynomialSplineConstraint(id, StateLin3d(initial_ee_W_.At(ee)), t));
  }

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

  // acceleration important b/c enforcing system dynamics only once at the
  // junction, so make sure second polynomial also respect that by making
  // its accelerations equal to the first.
  auto derivatives = {kPos, kVel, kAcc};
  junction_constraints->AddComponent(MakePolynomialJunctionConstraint(id::base_linear, derivatives));
  junction_constraints->AddComponent(MakePolynomialJunctionConstraint(id::base_angular, derivatives));

  // allow lifting/placing of endeffector with nonzero acceleration
  for (auto ee : params->robot_ee_) {
    std::string id_motion = id::endeffectors_motion+std::to_string(ee);
    junction_constraints->AddComponent(MakePolynomialJunctionConstraint(id_motion, {kPos, kVel}));

    std::string id_force = id::endeffector_force+std::to_string(ee);
//    junction_constraints->AddComponent(MakePolynomialJunctionConstraint(id_force, {kPos}, 4));
  }

  return junction_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakePolynomialJunctionConstraint (const std::string& poly_id,
                                                         const Derivatives& derivatives) const
{
//  auto poly = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars_->GetComponent(poly_id));
//  LinearSplineEquations equation_builder(*poly);
//  return std::make_shared<LinearEqualityConstraint>(opt_vars_, equation_builder.MakeJunction(derivatives), poly_id);
//
  return std::make_shared<SplineJunctionConstraint>(opt_vars_, poly_id, derivatives);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeDynamicConstraint() const
{
//  model_ = std::make_shared<LIPModel>();
  auto dynamic_model = std::make_shared<CentroidalModel>(params->GetMass(),
                                                         params->GetInertiaParameters(),
                                                         params->GetEECount());

  double dt = params->duration_polynomial_/params->n_constraints_per_poly_;
  auto constraint = std::make_shared<DynamicConstraint>(opt_vars_,
                                                        dynamic_model,
                                                        params->GetTotalTime(),
                                                        dt
                                                        );
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  auto rom_constraints = std::make_shared<Composite>("Range-of-Motion Constraints", true);
  double dt = 0.10;

  for (auto ee : params->robot_ee_) {
    auto c = std::make_shared<RangeOfMotionBox>(opt_vars_,
                                                dt,
                                                params->GetMaximumDeviationFromNominal(),
                                                params->GetNominalStanceInBase().At(ee),
                                                ee,
                                                params->GetTotalTime());

    rom_constraints->AddComponent(c);

//    // add timing constraint
//    auto contact_timing_constraints = std::make_shared<ContactConstraints>(opt_vars_,params->GetTotalTime(),dt,ee);
//    rom_constraints->AddComponent(contact_timing_constraints);
  }








  return rom_constraints;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeStancesConstraints () const
{
  auto stance_constraints = std::make_shared<Composite>("Stance Constraints", true);

  double t_start = 0.0;
  double t_end   = params->GetTotalTime();

  Eigen::Matrix3d w_R_b = AngularStateConverter::GetRotationMatrixBaseToWorld(final_base_.ang.p_);
  EndeffectorsPos nominal_B = params->GetNominalStanceInBase();

  for (auto ee : params->robot_ee_) {
    std::string id = id::endeffectors_motion+std::to_string(ee);
    stance_constraints->AddComponent(MakePolynomialSplineConstraint(id, StateLin3d(initial_ee_W_.At(ee)), t_start));

    Endeffectors<StateLin3d> endeffectors_final_W(nominal_B.GetCount());
    endeffectors_final_W.At(ee).p_ = final_base_.lin.p_ + w_R_b*nominal_B.At(ee);

    stance_constraints->AddComponent(MakePolynomialSplineConstraint(id, StateLin3d(endeffectors_final_W.At(ee)), t_end));
  }

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

