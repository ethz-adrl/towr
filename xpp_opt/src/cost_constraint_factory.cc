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

#include <ifopt/soft_constraint.h>

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

using namespace opt;


void
CostConstraintFactory::Init (const MotionParamsPtr& _params,
                             const HeightMap::Ptr& terrain,
                             const RobotModel& model,
                             const EndeffectorsPos& ee_pos,
                             const State3dEuler& initial_base,
                             const State3dEuler& final_base)
{
  params    = _params;
  terrain_  = terrain;
  model_    = model;

  initial_ee_W_ = ee_pos;
  initial_base_ = initial_base;
  final_base_ = final_base;
}

CostConstraintFactory::ContraintPtrVec
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

CostConstraintFactory::CostPtrVec
CostConstraintFactory::GetCost(const CostName& name, double weight) const
{
  switch (name) {
    case ForcesCostID:       return MakeForcesCost(weight);
//    case ComCostID:          return MakeMotionCost(weight);
//    case RangOfMotionCostID: return ToCost(MakeRangeOfMotionBoxConstraint(), weight);
    default: throw std::runtime_error("cost not defined!");
  }
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeStateConstraint () const
{
  ContraintPtrVec constraints;
//  auto constraints = std::make_shared<Composite>("State Constraints", false);



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
  constraints.push_back(std::make_shared<SplineStateConstraint>(id::base_linear, t0, initial_base_.lin, PosVelAcc_, XYZ_));
  constraints.push_back(std::make_shared<SplineStateConstraint>(id::base_linear, T,  final_base_.lin,   PosVelAcc_, XY_));
  constraints.push_back(std::make_shared<SplineStateConstraint>(id::base_linear, T,  final_base_.lin,   PosVelAcc_, Z_));

  // angular base motion
  constraints.push_back(std::make_shared<SplineStateConstraint>(id::base_angular, t0, initial_base_.ang, PosVelAcc_, XYZ_));
  constraints.push_back(std::make_shared<SplineStateConstraint>(id::base_angular, T,  final_base_.ang,   PosVelAcc_, XYZ_));
//  constraints.push_back(std::make_shared<SplineStateConstraint>(id::base_angular, T,  final_base_.ang,   VelAcc_, XYZ_));

  // junction constraints
  constraints.push_back(std::make_shared<SplineJunctionConstraint>(id::base_linear,  PosVelAcc_));
  constraints.push_back(std::make_shared<SplineJunctionConstraint>(id::base_angular, PosVelAcc_));



//  // endeffector constraints
//  for (auto ee : params->robot_ee_) {
//
//    auto spline_ee = Spline::BuildSpline(id::GetEEMotionId(ee), {});
//
//    // initial endeffectors constraints
//    // zmp_ replace these by normal variable bounds on the hermite-poly nodes
//    auto deriv_ee = {kPos}; // velocity and acceleration not yet implemented
//    auto c = std::make_shared<SplineStateConstraint>(spline_ee, t,
//                                                     VectorXd(initial_ee_W_.At(ee)),
//                                                     deriv_ee);
////    constraints.push_back(c);
//
//    // final endeffectors constraints
//    Eigen::Matrix3d w_R_b = AngularStateConverter::GetRotationMatrixBaseToWorld(final_base_.ang.p_);
//    EndeffectorsPos nominal_B = params->GetNominalStanceInBase();
//    VectorXd ee_pos_W = final_base_.lin.p_ + w_R_b*nominal_B.At(ee);
////    constraints.push_back(std::make_shared<SplineStateConstraint>(spline_ee, T, ee_pos_W, deriv_ee));
//
//  }

  return constraints;
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeBaseRangeOfMotionConstraint () const
{
  return {std::make_shared<BaseMotionConstraint>(*params)};
}

CostConstraintFactory::ContraintPtrVec
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
        dts_.push_back(t_node-eps); // this results in continuous acceleration along junctions
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

  auto constraint = std::make_shared<DynamicConstraint>(model_.dynamic_model_,
                                                        dts_
                                                        );
  return {constraint};
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeRangeOfMotionBoxConstraint () const
{
  ContraintPtrVec c;
//  auto c = std::make_shared<Composite>("Range-of-Motion Constraints", false);

  double T = params->GetTotalTime();

  for (auto ee : GetEEIDs()) {
    auto rom_constraints = std::make_shared<RangeOfMotionBox>(*params,
                                                              model_.kinematic_model_,
                                                              ee);
    c.push_back(rom_constraints);
  }


  return c;
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeTotalTimeConstraint () const
{
  ContraintPtrVec c;
//  auto c = std::make_shared<Composite>("TotalTimeConstraint", false);
  double T = params->GetTotalTime();

  for (auto ee : GetEEIDs()) {
    auto duration_constraint = std::make_shared<DurationConstraint>(T, ee);
    c.push_back(duration_constraint);
  }

  return c;
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeTerrainConstraint () const
{
  ContraintPtrVec constraints;
//  auto constraints = std::make_shared<Composite>("Terrain Constraints", false);

  for (auto ee : GetEEIDs()) {
    auto c = std::make_shared<TerrainConstraint>(terrain_, id::GetEEMotionId(ee));
    constraints.push_back(c);
  }

  return constraints;
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeForceConstraint () const
{
  ContraintPtrVec constraints;
//  auto constraints = std::make_shared<Composite>("Force Constraints", false);

  for (auto ee : GetEEIDs()) {
    auto c = std::make_shared<ForceConstraint>(terrain_,
                                               model_.dynamic_model_->GetForceLimit(),
                                               ee);
    constraints.push_back(c);
  }

  return constraints;
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeSwingConstraint () const
{
  ContraintPtrVec constraints;
//  auto constraints = std::make_shared<Composite>("Swing Constraints", false);

  for (auto ee : GetEEIDs()) {
    auto swing = std::make_shared<SwingConstraint>(id::GetEEMotionId(ee));
    constraints.push_back(swing);
  }

  return constraints;
}


CostConstraintFactory::CostPtrVec
CostConstraintFactory::MakeForcesCost(double weight) const
{
  CostPtrVec cost;
//  auto cost = std::make_shared<Composite>("Forces Cost", true);

  for (auto ee : GetEEIDs())
    cost.push_back(std::make_shared<NodeCost>(id::GetEEForceId(ee)));

  return cost;
}


//CostConstraintFactory::CostPtrVec
//CostConstraintFactory::MakeMotionCost(double weight) const
//{
//  CostPtrVec base_acc_cost;
////  auto base_acc_cost = std::make_shared<Composite>("Base Acceleration Costs", true);
//
//  VectorXd weight_xyz(kDim3d); weight_xyz << 1.0, 1.0, 1.0;
//  base_acc_cost.push_back(MakePolynomialCost(id::base_linear, weight_xyz, weight));
//
//  VectorXd weight_angular(kDim3d); weight_angular << 0.1, 0.1, 0.1;
//  base_acc_cost.push_back(MakePolynomialCost(id::base_angular, weight_angular, weight));
//
//  return base_acc_cost;
//
////  return std::make_shared<NodeCost>(id::base_linear);
//}

//CostConstraintFactory::CostPtr
//CostConstraintFactory::MakePolynomialCost (const std::string& poly_id,
//                                           const Vector3d& weight_dimensions,
//                                           double weight) const
//{
//  assert(false); /// not implemented at the moment
////  auto poly = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars_->GetComponent(poly_id));
////  LinearSplineEquations equation_builder(*poly);
////
////  Eigen::MatrixXd term = equation_builder.MakeCostMatrix(weight_dimensions, kAcc);
////
////  MatVec mv(term.rows(), term.cols());
////  mv.M = term;
////  mv.v.setZero();
////
////  return std::make_shared<QuadraticPolynomialCost>(mv, poly_id, weight);
//}

//CostConstraintFactory::CostPtrVec
//CostConstraintFactory::ToCost (const ComponentPtr& constraint, double weight) const
//{
//  return {std::make_shared<SoftConstraint>(constraint)};
//}

} /* namespace xpp */

