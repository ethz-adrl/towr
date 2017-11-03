/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <xpp_opt/cost_constraint_factory.h>

#include <cassert>
#include <initializer_list>
#include <stdexcept>

#include <xpp_solve/soft_constraint.h>

#include <xpp_opt/constraints/base_motion_constraint.h>
#include <xpp_opt/constraints/dynamic_constraint.h>
#include <xpp_opt/constraints/force_constraint.h>
#include <xpp_opt/constraints/range_of_motion_constraint.h>
#include <xpp_opt/constraints/spline_constraint.h>
#include <xpp_opt/constraints/swing_constraint.h>
#include <xpp_opt/constraints/terrain_constraint.h>
#include <xpp_opt/costs/node_cost.h>
#include <xpp_opt/models/dynamic_model.h>
#include <xpp_opt/variables/contact_schedule.h>
#include <xpp_opt/variables/variable_names.h>


namespace xpp {

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
                             const RobotModel& model,
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
    case BasePoly:       return MakeStateConstraint();
    case Dynamic:        return MakeDynamicConstraint();
    case EndeffectorRom: return MakeRangeOfMotionBoxConstraint();
    case BaseRom:        return MakeBaseRangeOfMotionConstraint();
    case TotalTime:      return MakeTotalTimeConstraint();
    case Terrain:        return MakeTerrainConstraint();
    case Force:          return MakeForceConstraint();
    case Swing:          return MakeSwingConstraint();
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
  auto constraints = std::make_shared<Composite>("State Constraints", true);



  // initial base constraints
  double t0 = 0.0; // initial time
  double T = params->GetTotalTime();
  auto Z_         = {Z};
  auto XY_        = {X,Y};
  auto XYZ_       = {X, Y, Z};
  auto Pos_       = {kPos};
  auto PosVel_    = {kPos, kVel};
  auto VelAcc_    = {kVel, kAcc};
  auto PosVelAcc_ = {kPos, kVel, kAcc};


  // linear base motion
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, id::base_linear, t0, initial_base_.lin, PosVelAcc_, XYZ_));
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, id::base_linear, T,  final_base_.lin,   PosVelAcc_, XYZ_));

  // angular base motion
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, id::base_angular, t0, initial_base_.ang, PosVelAcc_, XYZ_));
  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, id::base_angular, T,  final_base_.ang,   PosVelAcc_, XYZ_));
//  constraints->AddComponent(std::make_shared<SplineStateConstraint>(opt_vars_, id::base_angular, T,  final_base_.ang,   VelAcc_, XYZ_));

  // junction constraints
  constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_linear,  PosVelAcc_));
  constraints->AddComponent(std::make_shared<SplineJunctionConstraint>(opt_vars_, id::base_angular, PosVelAcc_));



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

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeBaseRangeOfMotionConstraint () const
{
  return std::make_shared<BaseMotionConstraint>(opt_vars_, *params);
}

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
                                                        model_.dynamic_model_,
                                                        dts_
                                                        );
  return constraint;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  auto c = std::make_shared<Composite>("Range-of-Motion Constraints", true);

  double T = params->GetTotalTime();

  for (auto ee : GetEEIDs()) {
    auto rom_constraints = std::make_shared<RangeOfMotionBox>(opt_vars_,
                                                              *params,
                                                              model_.kinematic_model_,
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

  for (auto ee : GetEEIDs()) {
    auto duration_constraint = std::make_shared<DurationConstraint>(opt_vars_, T, ee);
    c->AddComponent(duration_constraint);
  }

  return c;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeTerrainConstraint () const
{
  auto constraints = std::make_shared<Composite>("Terrain Constraints", true);

  for (auto ee : GetEEIDs()) {
    auto c = std::make_shared<TerrainConstraint>(terrain_,
                                                 opt_vars_,
                                                 id::GetEEMotionId(ee));
    constraints->AddComponent(c);
  }

  return constraints;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeForceConstraint () const
{
  auto constraints = std::make_shared<Composite>("Force Constraints", true);

  for (auto ee : GetEEIDs()) {
    auto c = std::make_shared<ForceConstraint>(terrain_,
                                               model_.dynamic_model_->GetForceLimit(),
                                               opt_vars_,
                                               id::GetEEForceId(ee),
                                               id::GetEEMotionId(ee));
    constraints->AddComponent(c);
  }

  return constraints;
}

CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeSwingConstraint () const
{
  auto constraints = std::make_shared<Composite>("Swing Constraints", true);

  for (auto ee : GetEEIDs()) {
    auto swing = std::make_shared<SwingConstraint>(opt_vars_,
                                                   id::GetEEMotionId(ee));
    constraints->AddComponent(swing);
  }

  return constraints;
}


CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeForcesCost(double weight) const
{
  auto cost = std::make_shared<Composite>("Forces Cost", false);

  for (auto ee : GetEEIDs()) {
    auto f_cost = std::make_shared<NodeCost>(opt_vars_, id::GetEEForceId(ee));
    cost->AddComponent(f_cost);
  }

  return cost;
}


CostConstraintFactory::ComponentPtr
CostConstraintFactory::MakeMotionCost(double weight) const
{
  auto base_acc_cost = std::make_shared<Composite>("Base Acceleration Costs", false);

  VectorXd weight_xyz(kDim3d); weight_xyz << 1.0, 1.0, 1.0;
  base_acc_cost->AddComponent(MakePolynomialCost(id::base_linear, weight_xyz, weight));

  VectorXd weight_angular(kDim3d); weight_angular << 0.1, 0.1, 0.1;
  base_acc_cost->AddComponent(MakePolynomialCost(id::base_angular, weight_angular, weight));

  return base_acc_cost;

//  return std::make_shared<NodeCost>(opt_vars_, id::base_linear);
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

} /* namespace xpp */

