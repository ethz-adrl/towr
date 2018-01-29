/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/nlp_factory.h>

#include <towr/variables/variable_names.h>
#include <towr/variables/base_nodes.h>
#include <towr/variables/phase_durations.h>

#include <towr/constraints/base_motion_constraint.h>
#include <towr/constraints/dynamic_constraint.h>
#include <towr/constraints/force_constraint.h>
#include <towr/constraints/range_of_motion_constraint.h>
#include <towr/constraints/swing_constraint.h>
#include <towr/constraints/terrain_constraint.h>
#include <towr/constraints/total_duration_constraint.h>

#include <towr/costs/node_cost.h>

namespace towr {


NlpFactory::VariablePtrVec
NlpFactory::GetVariableSets ()
{
  VariablePtrVec vars;

  auto base_motion = MakeBaseVariables();
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

std::vector<Nodes::Ptr>
NlpFactory::MakeBaseVariables () const
{
  std::vector<Nodes::Ptr> vars;

  int n_nodes = params_.GetBasePolyDurations().size() + 1;

  auto spline_lin = std::make_shared<BaseNodes>(n_nodes, id::base_lin_nodes);
  spline_lin->InitializeNodesTowardsGoal(initial_base_.lin.p(), final_base_.lin.p(), params_.t_total_);
  spline_lin->AddStartBound(kPos, {X,Y,Z}, initial_base_.lin.p());
  spline_lin->AddStartBound(kVel, {X,Y,Z}, initial_base_.lin.v());
  spline_lin->AddFinalBound(kPos, {X,Y}  , final_base_.lin.p());
  spline_lin->AddFinalBound(kVel, {X,Y,Z}, final_base_.lin.v());
  vars.push_back(spline_lin);

  auto spline_ang = std::make_shared<BaseNodes>(n_nodes,  id::base_ang_nodes);
  spline_ang->InitializeNodesTowardsGoal(initial_base_.ang.p(), final_base_.ang.p(), params_.t_total_);
  spline_ang->AddStartBound(kPos, {X,Y,Z}, initial_base_.ang.p());
  spline_ang->AddStartBound(kVel, {X,Y,Z}, initial_base_.ang.v());
  spline_ang->AddFinalBound(kPos,     {Z}, final_base_.ang.p());
  spline_ang->AddFinalBound(kVel, {X,Y,Z}, final_base_.ang.v());
  vars.push_back(spline_ang);

  return vars;
}

std::vector<PhaseNodes::Ptr>
NlpFactory::MakeEndeffectorVariables () const
{
  std::vector<PhaseNodes::Ptr> vars;

  // Endeffector Motions
  double T = params_.t_total_;
  for (int ee=0; ee<params_.GetEECount(); ee++) {

    auto nodes = std::make_shared<PhaseNodes>(params_.GetPhaseCount(ee),
                                              params_.ee_in_contact_at_start_.at(ee),
                                              id::EEMotionNodes(ee),
                                              params_.ee_polynomials_per_swing_phase_,
                                              PhaseNodes::Motion);

    double yaw = final_base_.ang.p().z();

    // smell adapt to desired yaw state
//    Eigen::Matrix3d w_R_b = GetQuaternionFromEulerZYX(yaw, 0.0, 0.0).toRotationMatrix();
    Eigen::Matrix3d w_R_b; w_R_b.setIdentity();

    Vector3d final_ee_pos_W = final_base_.lin.p() + w_R_b*model_.kinematic_model_->GetNominalStanceInBase().at(ee);



    nodes->InitializeNodesTowardsGoal(initial_ee_W_.at(ee), final_ee_pos_W, T);

    // actually initial Z position should be constrained as well...-.-
    nodes->AddStartBound(kPos, {X,Y}, initial_ee_W_.at(ee));

    // fix final endeffector position
//    bool step_taken = nodes->GetNodes().size() > 2;
//    if (step_taken) // otherwise overwrites start bound
//      nodes->AddFinalBound(kPos, {X,Y}, final_ee_pos_W);

    vars.push_back(nodes);
  }


  return vars;
}

std::vector<PhaseNodes::Ptr>
NlpFactory::MakeForceVariables () const
{
  std::vector<PhaseNodes::Ptr> vars;

  double T = params_.t_total_;
  for (int ee=0; ee<params_.GetEECount(); ee++) {

    auto nodes = std::make_shared<PhaseNodes>(params_.GetPhaseCount(ee),
                                              params_.ee_in_contact_at_start_.at(ee),
                                              id::EEForceNodes(ee),
                                              params_.force_polynomials_per_stance_phase_,
                                              PhaseNodes::Force);

    // initialize with mass of robot distributed equally on all legs
    double m = model_.dynamic_model_->m();
    double g = model_.dynamic_model_->g();


    Vector3d f_stance(0.0, 0.0, m*g/params_.GetEECount());
    nodes->InitializeNodesTowardsGoal(f_stance, f_stance, T);
    vars.push_back(nodes);
  }

  return vars;
}

std::vector<PhaseDurations::Ptr>
NlpFactory::MakeContactScheduleVariables () const
{
  std::vector<PhaseDurations::Ptr> vars;

  for (int ee=0; ee<params_.GetEECount(); ee++) {

    auto var = std::make_shared<PhaseDurations>(ee,
                                                params_.ee_phase_durations_.at(ee),
                                                params_.ee_in_contact_at_start_.at(ee),
                                                params_.min_phase_duration_,
                                                params_.max_phase_duration_);
    vars.push_back(var);
  }

  return vars;
}

NlpFactory::ContraintPtrVec
NlpFactory::GetConstraints() const
{
  ContraintPtrVec constraints;
  for (ConstraintName name : params_.constraints_)
    for (auto c : GetConstraint(name))
      constraints.push_back(c);

  return constraints;
}

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

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto rom_constraints = std::make_shared<RangeOfMotionConstraint>(model_.kinematic_model_,
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
  double T = params_.t_total_;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto duration_constraint = std::make_shared<TotalDurationConstraint>(T, ee);
    c.push_back(duration_constraint);
  }

  return c;
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeTerrainConstraint () const
{
  ContraintPtrVec constraints;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto c = std::make_shared<TerrainConstraint>(terrain_, id::EEMotionNodes(ee));
    constraints.push_back(c);
  }

  return constraints;
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeForceConstraint () const
{
  ContraintPtrVec constraints;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto c = std::make_shared<ForceConstraint>(terrain_,
                                               params_.force_limit_in_norm_,
                                               ee);
    constraints.push_back(c);
  }

  return constraints;
}

NlpFactory::ContraintPtrVec
NlpFactory::MakeSwingConstraint () const
{
  ContraintPtrVec constraints;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto swing = std::make_shared<SwingConstraint>(id::EEMotionNodes(ee));
    constraints.push_back(swing);
  }

  return constraints;
}

NlpFactory::ContraintPtrVec
NlpFactory::GetCosts() const
{
  ContraintPtrVec costs;
  for (const auto& pair : params_.costs_)
    for (auto c : GetCost(pair.first, pair.second))
      costs.push_back(c);

  return costs;
}

NlpFactory::CostPtrVec
NlpFactory::GetCost(const CostName& name, double weight) const
{
  switch (name) {
    case ForcesCostID:       return MakeForcesCost(weight);
    default: throw std::runtime_error("cost not defined!");
  }
}

NlpFactory::CostPtrVec
NlpFactory::MakeForcesCost(double weight) const
{
  CostPtrVec cost;

  for (int ee=0; ee<params_.GetEECount(); ee++)
    cost.push_back(std::make_shared<NodeCost>(id::EEForceNodes(ee), kPos, Z));

  return cost;
}

} /* namespace towr */
