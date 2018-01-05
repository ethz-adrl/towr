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

#include <xpp_opt/costs/soft_constraint.h>

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
  params_   = _params;
  terrain_  = terrain;
  model_    = model;

  initial_ee_W_ = ee_pos;
  initial_base_ = initial_base;
  final_base_ = final_base;
}

CostConstraintFactory::VariablePtrVec
CostConstraintFactory::GetVariableSets () const
{
  VariablePtrVec vars;

  auto base_rep = params_->GetBaseRepresentation();
  switch (base_rep) {
    case OptimizationParameters::PolyCoeff:  {
      auto v = MakeBaseVariablesCoeff();
      vars.insert(vars.end(), v.begin(), v.end());
      break;
    }
    case OptimizationParameters::CubicHermite: {
      auto v = MakeBaseVariablesHermite();
      vars.insert(vars.end(), v.begin(), v.end());
      break;
    }
    default:
      throw std::runtime_error("base variable type not not defined!");
  }


  auto ee_motion = MakeEndeffectorVariables();
  vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());


  auto ee_forces = MakeForceVariables();
  vars.insert(vars.end(), ee_forces.begin(), ee_forces.end());


  bool optimize_timings = params_->ConstraintExists(TotalTime);
  if (optimize_timings) {
    auto contact_schedule = MakeContactScheduleVariables(ee_motion, ee_forces);
    vars.insert(vars.end(), contact_schedule.begin(), contact_schedule.end());
  }

  return vars;
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
  double T = params_->GetTotalTime();
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
  return {std::make_shared<BaseMotionConstraint>(*params_)};
}

CostConstraintFactory::ContraintPtrVec
CostConstraintFactory::MakeDynamicConstraint() const
{
  auto base_poly_durations = params_->GetBasePolyDurations();
  std::vector<double> dts_;
  double t_node = 0.0;
  dts_ = {t_node};

  double eps = 1e-6; // assume all polynomials have equal duration
  for (int i=0; i<base_poly_durations.size()-1; ++i) {
    double d = base_poly_durations.at(i);
    t_node += d;

    switch (params_->GetBaseRepresentation()) {
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

  if (params_->GetBaseRepresentation() == OptimizationParameters::PolyCoeff)
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

  for (auto ee : GetEEIDs()) {
    auto rom_constraints = std::make_shared<RangeOfMotionBox>(*params_,
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
  double T = params_->GetTotalTime();

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

  for (auto ee : GetEEIDs()) {
    auto swing = std::make_shared<SwingConstraint>(id::GetEEMotionId(ee));
    constraints.push_back(swing);
  }

  return constraints;
}

CostConstraintFactory::VariablePtrVec
CostConstraintFactory::MakeBaseVariablesCoeff () const
{
  VariablePtrVec vars;

  int n_dim = initial_base_.lin.kNumDim;
  int order = params_->order_coeff_polys_;

  std::vector<double> base_spline_timings_ = params_->GetBasePolyDurations();
  auto coeff_spline_ang = std::make_shared<CoeffSpline>(id::base_angular, base_spline_timings_);
  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_ang = std::make_shared<Polynomial>(order, n_dim);
    auto var = std::make_shared<PolynomialVars>(id::base_angular+std::to_string(i), p_ang);
    vars.push_back(var);

    coeff_spline_ang->poly_vars_.push_back(var);
  }
  coeff_spline_ang->InitializeVariables(initial_base_.ang.p_, final_base_.ang.p_);
  vars.push_back(coeff_spline_ang); // add just for easy access later


  auto coeff_spline_lin = std::make_shared<CoeffSpline>(id::base_linear, base_spline_timings_);
  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_lin = std::make_shared<Polynomial>(order, n_dim);
    auto var = std::make_shared<PolynomialVars>(id::base_linear+std::to_string(i), p_lin);
    vars.push_back(var);
    coeff_spline_lin->poly_vars_.push_back(var);
  }
  coeff_spline_lin->InitializeVariables(initial_base_.lin.p_, final_base_.lin.p_);
  vars.push_back(coeff_spline_lin); // add just for easy access later


  return vars;
}

CostConstraintFactory::VariablePtrVec
CostConstraintFactory::MakeBaseVariablesHermite () const
{
  VariablePtrVec vars;

  int n_dim = initial_base_.lin.kNumDim;
  std::vector<double> base_spline_timings_ = params_->GetBasePolyDurations();

  auto linear  = std::make_tuple(id::base_linear,  initial_base_.lin, final_base_.lin);
  auto angular = std::make_tuple(id::base_angular, initial_base_.ang, final_base_.ang);

  for (auto tuple : {linear, angular}) {
    std::string id   = std::get<0>(tuple);
    StateLin3d init  = std::get<1>(tuple);
    StateLin3d final = std::get<2>(tuple);

    auto spline = std::make_shared<NodeValues>(init.kNumDim,  base_spline_timings_.size(), id);
    spline->InitializeVariables(init.p_, final.p_, base_spline_timings_);

    std::vector<int> dimensions = {X,Y,Z};
    spline->AddStartBound(kPos, dimensions, init.p_);
    spline->AddStartBound(kVel, dimensions, init.v_);

    spline->AddFinalBound(kVel, dimensions, final.v_);

    if (id == id::base_linear) {
      spline->AddFinalBound(kPos, {X,Y}, final.p_); // only xy, z given by terrain
      //      spline->SetBoundsAboveGround();
    }
    if (id == id::base_angular)
      spline->AddFinalBound(kPos, {Z}, final.p_); // roll, pitch, yaw bound



    //    // force intermediate jump
    //    if (id == id::base_linear) {
    //      Vector3d inter = (init.p_ + final.p_)/2.;
    //      inter.z() = 0.8;
    //      spline->AddIntermediateBound(kPos, inter);
    //    }
    //
    //    if (id == id::base_angular) {
    //      spline->AddIntermediateBound(kPos, Vector3d::Zero());
    //    }


    vars.push_back(spline);
  }

  //  auto spline_lin = std::make_shared<NodeValues>(n_dim,  base_spline_timings_.size(), id::base_linear);
  //  spline_lin->InitializeVariables(inital_base_.lin.p_, final_base_.lin.p_, base_spline_timings_);
  //  spline_lin->AddBound(0,   kPos, inital_base_.lin.p_);
  //  spline_lin->AddBound(0,   kVel, inital_base_.lin.v_);
  //  spline_lin->AddFinalBound(kPos,  final_base_.lin.p_);
  //  spline_lin->AddFinalBound(kVel,  final_base_.lin.v_);
  //  opt_variables_->AddComponent(spline_lin);
  //
  //
  //  auto spline_ang = std::make_shared<NodeValues>(n_dim,  base_spline_timings_.size(), id::base_angular);
  //  spline_ang->InitializeVariables(inital_base_.ang.p_, final_base_.ang.p_, base_spline_timings_);
  //  spline_ang->AddBound(0,   kPos, inital_base_.ang.p_);
  //  spline_ang->AddBound(0,   kVel, inital_base_.ang.v_);
  //  spline_ang->AddFinalBound(kPos,  final_base_.ang.p_);
  //  spline_ang->AddFinalBound(kVel,  final_base_.ang.v_);
  //  opt_variables_->AddComponent(spline_ang);

  return vars;
}

CostConstraintFactory::VariablePtrVec
CostConstraintFactory::MakeEndeffectorVariables () const
{
  VariablePtrVec vars;

  // Endeffector Motions
  double T = params_->GetTotalTime();
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    auto contact_schedule = model_.gait_generator_->GetContactSchedule(T, ee);

    auto ee_motion = std::make_shared<EEMotionNodes>(contact_schedule.size(),
                                                     model_.gait_generator_->IsInContactAtStart(ee),
                                                     id::GetEEMotionId(ee),
                                                     params_->ee_splines_per_swing_phase_);

    double yaw = final_base_.ang.p_.z();
    Eigen::Matrix3d w_R_b = GetQuaternionFromEulerZYX(yaw, 0.0, 0.0).toRotationMatrix();
    Vector3d final_ee_pos_W = final_base_.lin.p_ + w_R_b*model_.kinematic_model_->GetNominalStanceInBase().at(ee);



    ee_motion->InitializeVariables(initial_ee_W_.at(ee), final_ee_pos_W, contact_schedule);

    // actually initial Z position should be constrained as well...-.-
    ee_motion->AddStartBound(kPos, {X,Y}, initial_ee_W_.at(ee));

    bool step_taken = ee_motion->GetNodes().size() > 2;
    if (step_taken) // otherwise overwrites start bound
      ee_motion->AddFinalBound(kPos, {X,Y}, final_ee_pos_W);

    vars.push_back(ee_motion);

  }


  return vars;
}

CostConstraintFactory::VariablePtrVec
CostConstraintFactory::MakeForceVariables () const
{
  VariablePtrVec vars;

  double T = params_->GetTotalTime();
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    auto contact_schedule = model_.gait_generator_->GetContactSchedule(T, ee);

    auto nodes_forces = std::make_shared<EEForceNodes>(contact_schedule.size(),
                                                       model_.gait_generator_->IsInContactAtStart(ee),
                                                       id::GetEEForceId(ee),
                                                       params_->force_splines_per_stance_phase_);

    Vector3d f_stance(0.0, 0.0, model_.dynamic_model_->GetStandingZForce());
    nodes_forces->InitializeVariables(f_stance, f_stance, contact_schedule);
    vars.push_back(nodes_forces);
  }

  return vars;
}

CostConstraintFactory::VariablePtrVec
CostConstraintFactory::MakeContactScheduleVariables (const VariablePtrVec& ee_motion,
                                                     const VariablePtrVec& ee_force) const
{
  VariablePtrVec vars;

  double T = params_->GetTotalTime();
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    auto var = std::make_shared<ContactSchedule>(ee,
                                                 model_.gait_generator_->GetContactSchedule(T, ee),
                                                 params_->min_phase_duration_,
                                                 params_->max_phase_duration_);

    auto node_motion = std::dynamic_pointer_cast<PhaseNodes>(ee_motion.at(ee));
    auto node_force  = std::dynamic_pointer_cast<PhaseNodes>(ee_force.at(ee));

    // always update endeffector parameterization with the current durations
    var->AddObserver(node_motion);
    var->AddObserver(node_force);

    vars.push_back(var);
  }

  return vars;
}

CostConstraintFactory::CostPtrVec
CostConstraintFactory::MakeForcesCost(double weight) const
{
  CostPtrVec cost;

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

