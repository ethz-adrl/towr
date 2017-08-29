/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp/cost_constraint_factory.h>

#include <cassert>
#include <initializer_list>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/angular_state_converter.h>
#include <xpp/centroidal_model.h>
#include <xpp/constraints/dynamic_constraint.h>
#include <xpp/constraints/range_of_motion_constraint.h>
#include <xpp/constraints/spline_constraint.h>
#include <xpp/constraints/terrain_constraint.h>
#include <xpp/costs/node_cost.h>
#include <xpp/costs/soft_constraint.h>
#include <xpp/variables/contact_schedule.h>
#include <xpp/variables/spline.h>
#include <xpp/variables/variable_names.h>


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
                             const HeightMap::Ptr& terrain,
                             const DynamicModel::Ptr& model,
                             const EndeffectorsPos& ee_pos,
                             const State3dEuler& initial_base,
                             const State3dEuler& final_base)
{
  opt_vars_ = opt_vars;
  params    = _params;
  terrain_  = terrain;
  model_    = model;

  initial_ee_W_ = ee_pos;
  initial_base_ = initial_base;
  final_base_ = final_base;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::GetConstraint (ConstraintName name) const
{
  switch (name) {
    case BasePoly:  return MakeStateConstraint();
    case Dynamic:     return MakeDynamicConstraint();
    case RomBox:      return MakeRangeOfMotionBoxConstraint();
    case TotalTime:   return MakeTotalTimeConstraint();
    case Terrain:     return MakeTerrainConstraint();
    default: throw std::runtime_error("constraint not defined!");
  }
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::GetCost(const CostName& name, double weight) const
{
  switch (name) {
    case ForcesCostID:       return MakeForcesCost(weight);
    case ComCostID:          return MakeMotionCost(weight);
    case RangOfMotionCostID: return ToCost(MakeRangeOfMotionBoxConstraint(), weight);
    default: throw std::runtime_error("cost not defined!");
  }
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeStateConstraint () const
{
  auto constraints = std::make_shared<Composite>("State Initial Constraints", true);


  auto spline_lin = opt_vars_->GetComponent<Spline>(id::base_linear);
  auto spline_ang = opt_vars_->GetComponent<Spline>(id::base_angular);


  // initial base constraints
  double t = 0.0; // initial time
  auto dim = {X, Y, Z};
  auto derivs = {kPos, kVel};
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_lin, t, initial_base_.lin, derivs, dim));
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ang, t, initial_base_.ang, derivs, dim));


//  // final linear and angular velocities must be zero
  double T = params->GetTotalTime();

  dim = {X,Y};
  derivs = {kPos};
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_lin, T, final_base_.lin, derivs, dim));

  // final yaw
  dim = {Z};
  derivs = {kPos};
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ang, T, final_base_.ang, derivs, dim));

  dim = {X, Y, Z};
  derivs = {kVel};
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_lin, T, final_base_.lin, derivs, dim));
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ang, T, final_base_.ang, derivs, dim));



  // junction constraints
  derivs = {kPos, kVel, kAcc};
  constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_linear, derivs));
  constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_angular, derivs));



//  // endeffector constraints
//  for (auto ee : params->robot_ee_) {
//
//    auto spline_ee = Spline::BuildSpline(opt_vars_, id::GetEEMotionId(ee), {});
//
//    // initial endeffectors constraints
//    // zmp_ replace these by normal variable bounds on the hermite-poly nodes
//    auto deriv_ee = {kPos}; // velocity and acceleration not yet implemented
//    auto c = std::make_shared<SplineStateConstraint>(opt_vars_, spline_ee, t,
//                                                     VectorXd(initial_ee_W_.At(ee)),
//                                                     deriv_ee);
////    constraints->AddComponent(c);
//
//    // final endeffectors constraints
//    Eigen::Matrix3d w_R_b = AngularStateConverter::GetRotationMatrixBaseToWorld(final_base_.ang.p_);
//    EndeffectorsPos nominal_B = params->GetNominalStanceInBase();
//    VectorXd ee_pos_W = final_base_.lin.p_ + w_R_b*nominal_B.At(ee);
////    constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, spline_ee, T, ee_pos_W, deriv_ee));
//
//  }

  return constraints;
}


//CostConstraintFactory::ComponentPtr
//CostConstraintFactory::MakeJunctionConstraint () const
//{
//  auto junction_constraints = std::make_shared<Composite>("Junctions Constraints", true);
//
//  // acceleration important b/c enforcing system dynamics only once at the
//  // junction, so make sure second polynomial also respect that by making
//  // its accelerations equal to the first.
//  auto derivatives = {kPos, kVel, kAcc};
//
//  junction_constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_linear, derivatives));
//  junction_constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_angular, derivatives));
//
////  for (auto ee : params->robot_ee_) {
////    auto durations_ee = contact_schedule_->GetTimePerPhase(ee);
//
////    auto derivs_pos_vel = {kPos, kVel};
////    junction_constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::GetEEId(ee), durations_ee, derivs_pos_vel));
//
////  }
//
//  return junction_constraints;
//}



CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeDynamicConstraint() const
{
  auto base_poly_durations = params->GetBasePolyDurations();
  std::vector<double> dts_;
  double t_node = 0.0;
  dts_ = {t_node};

  double eps = 1e-6; // assume all polynomials have equal duration
  for (int i=0; i<base_poly_durations.size()-1; ++i) {
    double d = base_poly_durations.at(i);
    t_node += d;

    switch (params->GetBaseRepresentation()) {
      case OptimizationParameters::CubicHermite:
        dts_.push_back(t_node-eps); // this results in continous acceleration along junctions
        dts_.push_back(t_node+eps);
        break;
      case OptimizationParameters::PolyCoeff:
        dts_.push_back(t_node-d/2.); // enforce dynamics at center of node
        dts_.push_back(t_node);
        break;
      default:
        assert(false); // representation not defined
        break;
    }
  }

  double final_d = base_poly_durations.back();
  t_node += final_d;

  if (params->GetBaseRepresentation() == OptimizationParameters::PolyCoeff)
    dts_.push_back(t_node-final_d/2);

  dts_.push_back(t_node); // also ensure constraints at very last node/time.



  auto constraint = std::make_shared<DynamicConstraint>(opt_vars_,
                                                        model_,
                                                        dts_
                                                        );

  return constraint;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  auto c = std::make_shared<Composite>("Range-of-Motion Constraints", true);

  double T = params->GetTotalTime();

  for (auto ee : model_->GetEEIDs()) {
    auto rom_constraints = std::make_shared<RangeOfMotionBox>(opt_vars_,
                                                              params,
                                                              model_->GetNominalStanceInBase().At(ee),
                                                              model_->GetMaximumDeviationFromNominal(),
                                                              ee);

    c->AddComponent(rom_constraints);
  }


  return c;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeTotalTimeConstraint () const
{
  auto c = std::make_shared<Composite>("Range-of-Motion Constraints", true);
  double T = params->GetTotalTime();

  for (auto ee : model_->GetEEIDs()) {
    auto duration_constraint = std::make_shared<DurationConstraint>(opt_vars_, T, ee);
    c->AddComponent(duration_constraint);
  }

  return c;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeTerrainConstraint () const
{
  auto constraints = std::make_shared<Composite>("Terrain Constraints", true);

  for (auto ee : model_->GetEEIDs()) {
    auto c = std::make_shared<TerrainConstraint>(terrain_,
                                                 opt_vars_,
                                                 id::GetEEMotionId(ee));
    constraints->AddComponent(c);
  }

  return constraints;
}


CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeForcesCost(double weight) const
{
  auto cost = std::make_shared<Composite>("Forces Cost", false);

  for (auto ee : model_->GetEEIDs()) {
    auto f_cost = std::make_shared<NodeCost>(opt_vars_, id::GetEEForceId(ee));
    cost->AddComponent(f_cost);
  }

  return cost;
}


CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeMotionCost(double weight) const
{
//  auto base_acc_cost = std::make_shared<Composite>("Base Acceleration Costs", false);
//
//  VectorXd weight_xyz(kDim3d); weight_xyz << 1.0, 1.0, 1.0;
//  base_acc_cost->AddComponent(MakePolynomialCost(id::base_linear, weight_xyz, weight));
//
//  VectorXd weight_angular(kDim3d); weight_angular << 0.1, 0.1, 0.1;
//  base_acc_cost->AddComponent(MakePolynomialCost(id::base_angular, weight_angular, weight));
//
//  return base_acc_cost;

  return std::make_shared<NodeCost>(opt_vars_, id::base_linear);
}

CostConstraintFactory::ComponentPtr
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

CostConstraintFactory::ComponentPtr
CostConstraintFactory::ToCost (const ComponentPtr& constraint, double weight) const
{
  return std::make_shared<SoftConstraint>(constraint, weight);
}

} /* namespace opt */
} /* namespace xpp */

