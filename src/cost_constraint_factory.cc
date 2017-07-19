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

#include <xpp/opt/variables/spline.h>
#include <xpp/opt/variables/contact_schedule.h>
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

  contact_schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars_->GetComponent(id::contact_schedule));

}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::GetConstraint (ConstraintName name) const
{
  switch (name) {
    case State:       return MakeStateConstraint();
    case JunctionCom: return MakeJunctionConstraint();
    case Dynamic:     return MakeDynamicConstraint();
    case RomBox:      return MakeRangeOfMotionBoxConstraint();
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
    default: throw std::runtime_error("cost not defined!");
  }
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeStateConstraint () const
{
  auto state_constraints = std::make_shared<Composite>("State Initial Constraints", true);


  auto base_poly_durations = params->GetBasePolyDurations();

  auto derivs = {kPos, kVel, kAcc};


  auto spline_lin = Spline::BuildSpline(opt_vars_, id::base_linear, base_poly_durations);
  auto spline_ang = Spline::BuildSpline(opt_vars_, id::base_angular, base_poly_durations);


  // initial base constraints
  double t = 0.0; // initial time (local)
  state_constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_lin.GetPolynomials().front(), t, initial_base_.lin, derivs));
  state_constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ang.GetPolynomials().front(), t, initial_base_.ang, derivs));


  // final base constraints
  double T_local = base_poly_durations.back();
  state_constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_lin.GetPolynomials().back(), T_local, final_base_.lin, derivs));
  state_constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ang.GetPolynomials().back(), T_local, final_base_.ang, derivs));


  // endeffector constraints
  auto contact_schedule = std::dynamic_pointer_cast<ContactSchedule>(opt_vars_->GetComponent(id::contact_schedule));
  for (auto ee : params->robot_ee_) {

    auto durations_ee = contact_schedule->GetTimePerPhase(ee);
    auto spline_ee = Spline::BuildSpline(opt_vars_, id::GetEEId(ee), durations_ee);


    // initial endeffectors constraints
//    auto spline_ee = Spline::BuildSpline(opt_vars_, id::GetEEId(ee), contact_schedule_->GetTimePerPhase(ee));
    state_constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ee.GetPolynomials().front(), t, VectorXd(initial_ee_W_.At(ee)), derivs));


    // final endeffectors constraints
    Eigen::Matrix3d w_R_b = AngularStateConverter::GetRotationMatrixBaseToWorld(final_base_.ang.p_);
    EndeffectorsPos nominal_B = params->GetNominalStanceInBase();
    Endeffectors<StateLin3d> endeffectors_final_W(nominal_B.GetCount());
    endeffectors_final_W.At(ee).p_ = final_base_.lin.p_ + w_R_b*nominal_B.At(ee);
    state_constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ee.GetPolynomials().back(), durations_ee.back(), endeffectors_final_W.At(ee), derivs));

  }

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

  auto durations_base = params->GetBasePolyDurations();
  junction_constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_linear, durations_base, derivatives));
  junction_constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_angular, durations_base, derivatives));

  for (auto ee : params->robot_ee_) {
    auto durations_ee = contact_schedule_->GetTimePerPhase(ee);

    auto derivs_pos_vel = {kPos, kVel};
    junction_constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::GetEEId(ee), durations_ee, derivs_pos_vel));

  }

  return junction_constraints;
}



CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeDynamicConstraint() const
{
//  model_ = std::make_shared<LIPModel>();
  auto dynamic_model = std::make_shared<CentroidalModel>(params->GetMass(),
                                                         params->GetInertiaParameters(),
                                                         params->GetEECount());

  double dt = params->dt_base_polynomial_/params->n_constraints_per_poly_;
  auto constraint = std::make_shared<DynamicConstraint>(opt_vars_,
                                                        dynamic_model,
                                                        params->GetBasePolyDurations(),
                                                        params->GetTotalTime(),
                                                        dt
                                                        );
  return constraint;
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  auto rom_constraints = std::make_shared<Composite>("Range-of-Motion Constraints", true);

  for (auto ee : params->robot_ee_) {
    auto c = std::make_shared<RangeOfMotionBox>(opt_vars_,
                                                params,
                                                contact_schedule_->GetTimePerPhase(ee),
                                                ee);

    rom_constraints->AddComponent(c);

//    // add timing constraint
//    auto contact_timing_constraints = std::make_shared<ContactConstraints>(opt_vars_,params->GetTotalTime(),dt,ee);
//    rom_constraints->AddComponent(contact_timing_constraints);
  }


  return rom_constraints;
}


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
  assert(false); /// not implemented at the moment
//  auto poly = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars_->GetComponent(poly_id));
//  LinearSplineEquations equation_builder(*poly);
//
//  Eigen::MatrixXd term = equation_builder.MakeCostMatrix(weight_dimensions, kAcc);
//
//  MatVec mv(term.rows(), term.cols());
//  mv.M = term;
//  mv.v.setZero();
//
//  return std::make_shared<QuadraticPolynomialCost>(opt_vars_, mv, poly_id, weight);
}

CostConstraintFactory::ConstraintPtr
CostConstraintFactory::ToCost (const ConstraintPtr& constraint, double weight) const
{
  return std::make_shared<SoftConstraint>(constraint, weight);
}

} /* namespace opt */
} /* namespace xpp */

