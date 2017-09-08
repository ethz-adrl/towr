/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/motion_optimizer_facade.h>

#include <algorithm>
#include <cassert>
#include <deque>
#include <Eigen/Dense>
#include <string>
#include <utility>

#include <xpp/angular_state_converter.h>
#include <xpp/cartesian_declarations.h>
#include <xpp/cost_constraint_factory.h>
#include <xpp/polynomial.h>
#include <xpp/variables/coeff_spline.h>
#include <xpp/variables/contact_schedule.h>
#include <xpp/variables/phase_nodes.h>
#include <xpp/variables/variable_names.h>
#include <xpp/ipopt_adapter.h>
#include <xpp/snopt_adapter.h>

namespace xpp {
namespace opt {

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  params_  = std::make_shared<OptimizationParameters>();
  model_   = std::make_shared<AnymalModel>();
  terrain_ = std::make_shared<FlatGround>();

  BuildDefaultInitialState();
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
}

void
MotionOptimizerFacade::BuildDefaultInitialState ()
{
  auto p_nom_B = model_->GetNominalStanceInBase();

  inital_base_.lin.p_ << 0.0, 0.0, -p_nom_B.At(E0).z();
  inital_base_.ang.p_ << 0.0, 0.0, 0.0; // euler (roll, pitch, yaw)

  initial_ee_W_.SetCount(model_->GetEECount());
  for (auto ee : initial_ee_W_.GetEEsOrdered()) {
    initial_ee_W_.At(ee) = p_nom_B.At(ee) + inital_base_.lin.p_;
    initial_ee_W_.At(ee).z() = 0.0;
  }
}

MotionOptimizerFacade::OptimizationVariablesPtr
MotionOptimizerFacade::BuildVariables () const
{
  auto opt_variables = std::make_shared<Composite>("nlp_variables", true);
  opt_variables->ClearComponents();


  switch (params_->GetBaseRepresentation()) {
    case OptimizationParameters::CubicHermite:
      SetBaseRepresentationHermite(opt_variables);
      break;
    case OptimizationParameters::PolyCoeff:
      SetBaseRepresentationCoeff(opt_variables);
      break;
    default:
      assert(false); // representation not defined
      break;
  }


  std::vector<std::shared_ptr<ContactSchedule>> contact_schedule;
  for (auto ee : model_->GetEEIDs()) {
    contact_schedule.push_back(std::make_shared<ContactSchedule>(ee,
                                                                 params_->GetTotalTime(),
                                                                 model_->GetNormalizedInitialTimings(ee),
                                                                 params_->min_phase_duration_,
                                                                 params_->max_phase_duration_));
  }


  // Endeffector Motions
  for (auto ee : model_->GetEEIDs()) {
    auto ee_motion = std::make_shared<EEMotionNodes>(contact_schedule.at(ee)->GetContactSequence(),
                                                     id::GetEEMotionId(ee),
                                                     params_->ee_splines_per_swing_phase_);

    double yaw = final_base_.ang.p_.z();
    Eigen::Matrix3d w_R_b = GetQuaternionFromEulerZYX(yaw, 0.0, 0.0).toRotationMatrix();
    Vector3d final_ee_pos_W = final_base_.lin.p_ + w_R_b*model_->GetNominalStanceInBase().At(ee);



    ee_motion->InitializeVariables(initial_ee_W_.At(ee), final_ee_pos_W, contact_schedule.at(ee)->GetTimePerPhase());
    ee_motion->AddStartBound(kPos, {X,Y}, initial_ee_W_.At(ee));   // only xy, z given by terrain

    //spring_clean_ fixed final footholds
    ee_motion->AddFinalBound(kPos, {X,Y}, final_ee_pos_W);
    opt_variables->AddComponent(ee_motion);
  }

  // Endeffector Forces
  for (auto ee : model_->GetEEIDs()) {
    auto nodes_forces = std::make_shared<EEForceNodes>(contact_schedule.at(ee)->GetContactSequence(),
                                                     id::GetEEForceId(ee),
                                                     params_->force_splines_per_stance_phase_);

    Vector3d f_stance(0.0, 0.0, model_->GetStandingZForce());
    nodes_forces->InitializeVariables(f_stance, f_stance, contact_schedule.at(ee)->GetTimePerPhase());
    opt_variables->AddComponent(nodes_forces);
  }


  // make endeffector motion and forces dependent on durations
  bool optimize_timings = params_->ConstraintExists(TotalTime);
  for (auto ee : model_->GetEEIDs()) {

    opt_variables->AddComponent(contact_schedule.at(ee), optimize_timings);

    if (optimize_timings) {
      contact_schedule.at(ee)->AddObserver(opt_variables->GetComponent<PhaseNodes>(id::GetEEMotionId(ee)));
      contact_schedule.at(ee)->AddObserver(opt_variables->GetComponent<PhaseNodes>(id::GetEEForceId(ee)));
    }
  }


  opt_variables->Print();
  return opt_variables;
}

void
MotionOptimizerFacade::SetBaseRepresentationCoeff (OptimizationVariablesPtr& opt_variables) const
{
  int n_dim = inital_base_.lin.kNumDim;
  int order = params_->order_coeff_polys_;

  std::vector<double> base_spline_timings_ = params_->GetBasePolyDurations();
  auto coeff_spline_ang = std::make_shared<CoeffSpline>(id::base_angular, base_spline_timings_);
  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_ang = std::make_shared<Polynomial>(order, n_dim);
    auto var = std::make_shared<PolynomialVars>(id::base_angular+std::to_string(i), p_ang);
    opt_variables->AddComponent(var);

    coeff_spline_ang->poly_vars_.push_back(var);
  }
  coeff_spline_ang->InitializeVariables(inital_base_.ang.p_, final_base_.ang.p_);
  opt_variables->AddComponent(coeff_spline_ang, false); // add just for easy access later


  auto coeff_spline_lin = std::make_shared<CoeffSpline>(id::base_linear, base_spline_timings_);
  for (int i=0; i<base_spline_timings_.size(); ++i) {
    auto p_lin = std::make_shared<Polynomial>(order, n_dim);
    auto var = std::make_shared<PolynomialVars>(id::base_linear+std::to_string(i), p_lin);
    opt_variables->AddComponent(var);
    coeff_spline_lin->poly_vars_.push_back(var);
  }
  coeff_spline_lin->InitializeVariables(inital_base_.lin.p_, final_base_.lin.p_);
  opt_variables->AddComponent(coeff_spline_lin, false); // add just for easy access later
}

void
MotionOptimizerFacade::SetBaseRepresentationHermite (OptimizationVariablesPtr& opt_variables_) const
{
  int n_dim = inital_base_.lin.kNumDim;
  std::vector<double> base_spline_timings_ = params_->GetBasePolyDurations();

  auto linear  = std::make_tuple(id::base_linear,  inital_base_.lin, final_base_.lin);
  auto angular = std::make_tuple(id::base_angular, inital_base_.ang, final_base_.ang);

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


    opt_variables_->AddComponent(spline);
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
}

void
MotionOptimizerFacade::BuildCostConstraints(const OptimizationVariablesPtr& opt_variables)
{
  CostConstraintFactory factory;
  factory.Init(opt_variables, params_, terrain_, model_,
               initial_ee_W_, inital_base_, final_base_);

  auto constraints = std::make_unique<Composite>("constraints", true);
  for (ConstraintName name : params_->GetUsedConstraints())
    constraints->AddComponent(factory.GetConstraint(name));

  constraints->Print();
  nlp.AddConstraint(std::move(constraints));


  auto costs = std::make_unique<Composite>("costs", false);
  for (const auto& pair : params_->GetCostWeights())
    costs->AddComponent(factory.GetCost(pair.first, pair.second));

  costs->Print();
  nlp.AddCost(std::move(costs));
}

void
MotionOptimizerFacade::SolveProblem ()
{
  auto variables = BuildVariables();
  nlp.Init(variables);

  BuildCostConstraints(variables);

  switch (nlp_solver_) {
    case Ipopt:   IpoptAdapter::Solve(nlp); break;
    case Snopt:   SnoptAdapter::Solve(nlp); break;
    default: assert(false); // solver not implemented
  }

  nlp.PrintCurrent();
}

std::vector<MotionOptimizerFacade::RobotStateVec>
MotionOptimizerFacade::GetIntermediateSolutions (double dt) const
{
  std::vector<RobotStateVec> trajectories;

  for (int iter=0; iter<nlp.GetIterationCount(); ++iter) {
    auto opt_var = nlp.GetOptVariables(iter);
    trajectories.push_back(GetTrajectory(opt_var, dt));
  }

  return trajectories;
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt) const
{
  auto opt_var = nlp.GetOptVariables();
  return GetTrajectory(opt_var, dt);
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (const OptimizationVariablesPtr& vars,
                                        double dt) const
{
  RobotStateVec trajectory;
  double t=0.0;
  double T = vars->GetComponent<ContactSchedule>(id::GetEEScheduleId(E0))->GetTotalTime();
  while (t<=T+1e-5) {

    RobotStateCartesian state(model_->GetEECount());

    state.base_.lin = vars->GetComponent<Spline>(id::base_linear)->GetPoint(t);
    state.base_.ang = AngularStateConverter::GetState(vars->GetComponent<Spline>(id::base_angular)->GetPoint(t));

    for (auto ee : state.ee_motion_.GetEEsOrdered()) {
      state.ee_contact_.At(ee) = vars->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee))->IsInContact(t);
      state.ee_motion_.At(ee)  = vars->GetComponent<Spline>(id::GetEEMotionId(ee))->GetPoint(t);
      state.ee_forces_.At(ee)  = vars->GetComponent<Spline>(id::GetEEForceId(ee))->GetPoint(t).p_;
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}

void
MotionOptimizerFacade::SetFinalState (const StateLin3d& lin,
                                      const StateLin3d& ang)
{
  final_base_.ang = ang;
  final_base_.lin = lin;

  // height depends on terrain
  double z_terrain = terrain_->GetHeight(lin.p_.x(), lin.p_.y());
  double z_nominal_B = model_->GetNominalStanceInBase().At(E0).z();
  final_base_.lin.p_.z() = z_terrain - z_nominal_B;
}

} /* namespace opt */
} /* namespace xpp */


