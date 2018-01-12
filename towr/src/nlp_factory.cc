/**
 @file    constraint_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Brief description
 */

#include <towr/nlp_factory.h>

#include <towr/variables/variable_names.h>
#include <towr/variables/contact_schedule.h>

#include <towr/constraints/base_motion_constraint.h>
#include <towr/constraints/dynamic_constraint.h>
#include <towr/constraints/force_constraint.h>
#include <towr/constraints/range_of_motion_constraint.h>
#include <towr/constraints/swing_constraint.h>
#include <towr/constraints/terrain_constraint.h>
#include <towr/constraints/total_duration_constraint.h>

#include <towr/costs/node_cost.h>
#include <towr/models/dynamic_model.h>


namespace towr {

using namespace ifopt;
using namespace xpp;


void
NlpFactory::Init (const OptimizationParameters& params,
                  const HeightMap::Ptr& terrain,
                  const RobotModel& model,
                  const EndeffectorsPos& ee_pos,
                  const State3dEuler& initial_base,
                  const State3dEuler& final_base)
{
  params_   = params;
  terrain_  = terrain;
  model_    = model;

  initial_ee_W_ = ee_pos;
  initial_base_ = initial_base;
  final_base_ = final_base;
}

// smell make separate class just for variables
NlpFactory::VariablePtrVec
NlpFactory::GetVariableSets () const
{
  VariablePtrVec vars;

  auto base_motion = MakeBaseVariablesHermite();
  vars.insert(vars.end(), base_motion.begin(), base_motion.end());

  auto ee_motion = MakeEndeffectorVariables();
  vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());

  auto ee_force = MakeForceVariables();
  vars.insert(vars.end(), ee_force.begin(), ee_force.end());

  auto contact_schedule = MakeContactScheduleVariables();
  if (params_.OptimizeTimings()) {
    vars.insert(vars.end(), contact_schedule.begin(), contact_schedule.end());
  }

  // stores these readily constructed spline, independent of whether the
  // nodes and durations these depend on are optimized over
  spline_holder_ = SplineHolder(base_motion.at(0), // linear
                                base_motion.at(1), // angular
                                params_.GetBasePolyDurations(),
                                ee_motion,
                                ee_force,
                                contact_schedule,
                                params_.OptimizeTimings());


  return vars;
}

std::vector<NodeValues::Ptr>
NlpFactory::MakeBaseVariablesHermite () const
{
  std::vector<NodeValues::Ptr> vars;

  int n_dim = initial_base_.lin.kNumDim;
  int n_nodes = params_.GetBasePolyDurations().size() + 1;

  auto linear  = std::make_tuple(id::base_lin_nodes,  initial_base_.lin, final_base_.lin);
  auto angular = std::make_tuple(id::base_ang_nodes, initial_base_.ang, final_base_.ang);

  for (auto tuple : {linear, angular}) {
    std::string id   = std::get<0>(tuple);
    StateLin3d init  = std::get<1>(tuple);
    StateLin3d final = std::get<2>(tuple);

    auto nodes = std::make_shared<NodeValues>(init.kNumDim,  n_nodes, id);
    nodes->InitializeNodes(init.p_, final.p_, params_.GetTotalTime());

    std::vector<int> dimensions = {X,Y,Z};
    nodes->AddStartBound(kPos, dimensions, init.p_);
    nodes->AddStartBound(kVel, dimensions, init.v_);

    nodes->AddFinalBound(kVel, dimensions, final.v_);

    if (id == id::base_lin_nodes) {
      nodes->AddFinalBound(kPos, {X,Y}, final.p_); // only xy, z given by terrain
      //      spline->SetBoundsAboveGround();
    }
    if (id == id::base_ang_nodes)
      nodes->AddFinalBound(kPos, {Z}, final.p_); // roll, pitch, yaw bound



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


    vars.push_back(nodes);
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

std::vector<NodeValues::Ptr>
NlpFactory::MakeEndeffectorVariables () const
{
  std::vector<NodeValues::Ptr> vars;

  // Endeffector Motions
  double T = params_.GetTotalTime();
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    auto contact_schedule = model_.gait_generator_->GetContactSchedule(T, ee);

    auto nodes = std::make_shared<PhaseNodes>(contact_schedule.size(),
                                              model_.gait_generator_->IsInContactAtStart(ee),
                                              id::EEMotionNodes(ee),
                                              params_.ee_splines_per_swing_phase_,
                                              PhaseNodes::Motion);

    double yaw = final_base_.ang.p_.z();
    Eigen::Matrix3d w_R_b = GetQuaternionFromEulerZYX(yaw, 0.0, 0.0).toRotationMatrix();
    Vector3d final_ee_pos_W = final_base_.lin.p_ + w_R_b*model_.kinematic_model_->GetNominalStanceInBase().at(ee);



    nodes->InitializeNodes(initial_ee_W_.at(ee), final_ee_pos_W, T);

    // actually initial Z position should be constrained as well...-.-
    nodes->AddStartBound(kPos, {X,Y}, initial_ee_W_.at(ee));

    bool step_taken = nodes->GetNodes().size() > 2;
    if (step_taken) // otherwise overwrites start bound
      nodes->AddFinalBound(kPos, {X,Y}, final_ee_pos_W);

    vars.push_back(nodes);

  }


  return vars;
}

std::vector<NodeValues::Ptr>
NlpFactory::MakeForceVariables () const
{
  std::vector<NodeValues::Ptr> vars;

  double T = params_.GetTotalTime();
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    auto contact_schedule = model_.gait_generator_->GetContactSchedule(T, ee);

    auto nodes = std::make_shared<PhaseNodes>(contact_schedule.size(),
                                              model_.gait_generator_->IsInContactAtStart(ee),
                                              id::EEForceNodes(ee),
                                              params_.force_splines_per_stance_phase_,
                                              PhaseNodes::Force);

    Vector3d f_stance(0.0, 0.0, model_.dynamic_model_->GetStandingZForce());
    nodes->InitializeNodes(f_stance, f_stance, T);
    vars.push_back(nodes);
  }

  return vars;
}

std::vector<ContactSchedule::Ptr>
NlpFactory::MakeContactScheduleVariables () const
{
  std::vector<ContactSchedule::Ptr> vars;

  double T = params_.GetTotalTime();
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {

    auto var = std::make_shared<ContactSchedule>(ee,
                                                 model_.gait_generator_->GetContactSchedule(T, ee),
                                                 params_.min_phase_duration_,
                                                 params_.max_phase_duration_);
    vars.push_back(var);
  }

  return vars;
}



////////  constraints  ////////


NlpFactory::ContraintPtrVec
NlpFactory::GetConstraint (ConstraintName name) const
{
  switch (name) {
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

NlpFactory::CostPtrVec
NlpFactory::GetCost(const CostName& name, double weight) const
{
  switch (name) {
    case ForcesCostID:       return MakeForcesCost(weight);
//    case ComCostID:          return MakeMotionCost(weight);
//    case RangOfMotionCostID: return ToCost(MakeRangeOfMotionBoxConstraint(), weight);
    default: throw std::runtime_error("cost not defined!");
  }
}




NlpFactory::ContraintPtrVec
NlpFactory::MakeBaseRangeOfMotionConstraint () const
{
  return {std::make_shared<BaseMotionConstraint>(params_, spline_holder_)};
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeDynamicConstraint() const
{
  auto base_poly_durations = params_.GetBasePolyDurations();
  std::vector<double> dts_;
  double t_node = 0.0;
  dts_ = {t_node};

  double eps = 1e-6; // assume all polynomials have equal duration
  for (int i=0; i<base_poly_durations.size()-1; ++i) {
    double d = base_poly_durations.at(i);
    t_node += d;

    dts_.push_back(t_node-eps); // this results in continuous acceleration along junctions
    dts_.push_back(t_node+eps);
  }

  double final_d = base_poly_durations.back();
  t_node += final_d;

  dts_.push_back(t_node); // also ensure constraints at very last node/time.

  auto constraint = std::make_shared<DynamicConstraint>(model_.dynamic_model_,
                                                        dts_,
                                                        spline_holder_);
  return {constraint};
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeRangeOfMotionBoxConstraint () const
{
  ContraintPtrVec c;

  for (auto ee : GetEEIDs()) {
    auto rom_constraints = std::make_shared<RangeOfMotionBox>(model_.kinematic_model_,
                                                              params_,
                                                              ee,
                                                              spline_holder_);
    c.push_back(rom_constraints);
  }

  return c;
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeTotalTimeConstraint () const
{
  ContraintPtrVec c;
  double T = params_.GetTotalTime();

  for (auto ee : GetEEIDs()) {
    auto duration_constraint = std::make_shared<TotalDurationConstraint>(T, ee);
    c.push_back(duration_constraint);
  }

  return c;
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeTerrainConstraint () const
{
  ContraintPtrVec constraints;

  for (auto ee : GetEEIDs()) {
    auto c = std::make_shared<TerrainConstraint>(terrain_, id::EEMotionNodes(ee));
    constraints.push_back(c);
  }

  return constraints;
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeForceConstraint () const
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

NlpFactory::ContraintPtrVec
NlpFactory::MakeSwingConstraint () const
{
  ContraintPtrVec constraints;

  for (auto ee : GetEEIDs()) {
    auto swing = std::make_shared<SwingConstraint>(id::EEMotionNodes(ee));
    constraints.push_back(swing);
  }

  return constraints;
}


NlpFactory::CostPtrVec
NlpFactory::MakeForcesCost(double weight) const
{
  CostPtrVec cost;

  for (auto ee : GetEEIDs())
    cost.push_back(std::make_shared<NodeCost>(id::EEForceNodes(ee)));

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

} /* namespace towr */

