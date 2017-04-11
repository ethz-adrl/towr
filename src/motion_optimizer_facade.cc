/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/endeffectors_motion.h>
#include <xpp/opt/endeffector_load.h>
#include <xpp/opt/center_of_pressure.h>
#include <xpp/opt/contact_schedule.h>
#include <xpp/opt/motion_factory.h>
#include <xpp/opt/base_motion.h>
#include <xpp/optimization_variables_container.h>


#include <xpp/opt/cost_constraint_factory.h>


#include <xpp/ipopt_adapter.h>
#include <xpp/snopt_adapter.h>


namespace xpp {
namespace opt {

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  opt_variables_ = std::make_shared<OptimizationVariablesContainer>();

  // create corresponding heap object for each of the member pointers
  // zmp_ these should be normal objects owned by NLP.
  costs_         = std::make_shared<CostContainer>();
  constraints_   = std::make_shared<ConstraintContainer>();

  // zmp_ doesn't have to be a pointer
  nlp_ = std::make_shared<NLP>();
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
  // TODO Auto-generated destructor stub
}

void
MotionOptimizerFacade::BuildDefaultStartStance (const MotionParameters& params)
{
  State3d base;
  base.lin.p.z() = params.geom_walking_height_;
  EndeffectorsBool contact_state(params.robot_ee_.size());
  contact_state.SetAll(true);

  start_geom_.SetBase(base);
  start_geom_.SetContactState(contact_state);
  start_geom_.SetEEState(kPos, params.GetNominalStanceInBase());
}

void
MotionOptimizerFacade::OptimizeMotion (NlpSolver solver)
{
  auto goal_com = goal_geom_;
  goal_com.p += motion_parameters_->offset_geom_to_com_;


  // initialize the contact schedule
  auto contact_schedule = std::make_shared<ContactSchedule>(motion_parameters_->GetOneCycle());

  // initialize the ee_motion with the fixed parameters
  auto ee_motion = std::make_shared<EndeffectorsMotion>(start_geom_.GetEEPos(),*contact_schedule);

  double T = motion_parameters_->GetTotalTime();
  double com_height = motion_parameters_->geom_walking_height_
                    + motion_parameters_->offset_geom_to_com_.z();
  auto com_motion = MotionFactory::CreateComMotion(T,
                                                   motion_parameters_->polynomials_per_second_,
                                                   com_height);
  com_motion->SetOffsetGeomToCom(motion_parameters_->offset_geom_to_com_);


  double parameter_dt = motion_parameters_->dt_nodes_;
  auto load = std::make_shared<EndeffectorLoad>(*ee_motion, parameter_dt, T);
  auto cop  = std::make_shared<CenterOfPressure>(parameter_dt,T);

  opt_variables_->ClearVariables();
  opt_variables_->AddVariableSet(com_motion);
  opt_variables_->AddVariableSet(ee_motion);
  opt_variables_->AddVariableSet(load);
  opt_variables_->AddVariableSet(cop);
  opt_variables_->AddVariableSet(contact_schedule);


  SolveProblem(solver);

//  nlp_facade_.OptimizeMotion(start_geom_,
//                             goal_com.Get2D(),
//                             opt_variables_,
//                             motion_parameters_,
//                             solver);
}

void
MotionOptimizerFacade::SolveProblem (NlpSolver solver)
{
  CostConstraintFactory factory;
  factory.Init(opt_variables_, motion_parameters_, start_geom_, goal_geom_.Get2D());

  constraints_->ClearConstraints();
  for (ConstraintName name : motion_parameters_->GetUsedConstraints()) {
    constraints_->AddConstraint(factory.GetConstraint(name));
  }

  costs_->ClearCosts();
  for (const auto& pair : motion_parameters_->GetCostWeights()) {
    CostName name = pair.first;
    double weight = pair.second;
    costs_->AddCost(factory.GetCost(name), weight);
  }

  nlp_->Init(opt_variables_, costs_, constraints_);

  SolveNlp(solver);
}






MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt)
{
  RobotStateVec trajectory;

  auto base_motion      = std::dynamic_pointer_cast<BaseMotion>(opt_variables_->GetSet("base_motion"));
  auto ee_motion        = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_variables_->GetSet("endeffectors_motion"));
  auto contact_schedule = std::dynamic_pointer_cast<ContactSchedule>(opt_variables_->GetSet("contact_schedule"));

  double t=0.0;
  double T = motion_parameters_->GetTotalTime();
  while (t<T) {

    RobotStateCartesian state(start_geom_.GetEECount());
    state.SetBase(base_motion->GetBase(t));
    state.SetEEState(ee_motion->GetEndeffectors(t));
    state.SetContactState(contact_schedule->IsInContact(t));
    state.SetTime(t);

    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}

void
MotionOptimizerFacade::SetMotionParameters (const MotionParametersPtr& params)
{
  motion_parameters_ = params;
}

void
MotionOptimizerFacade::SolveNlp (NlpSolver solver)
{
  switch (solver) {
    case Ipopt:
      SolveIpopt();
      break;
    case Snopt:
      SolveSnopt();
      break;
    default:
      throw std::runtime_error("solver not implemented");
  }
}

void
MotionOptimizerFacade::SolveSnopt ()
{
  auto snopt_problem = SnoptAdapter::GetInstance();
  snopt_problem->SetNLP(nlp_);
  snopt_problem->Init();
  int Cold = 0, Basis = 1, Warm = 2;
  snopt_problem->SolveSQP(Cold);
}

void
MotionOptimizerFacade::SolveIpopt ()
{
  using namespace Ipopt;
  using IpoptPtr            = SmartPtr<TNLP>;
  using IpoptApplicationPtr = SmartPtr<IpoptApplication>;

  // initialize the ipopt solver
  IpoptApplicationPtr ipopt_app_ = new IpoptApplication();
  ipopt_app_->RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt

  //  ipopt_solver_->Options()->SetNumericValue("max_cpu_time", max_cpu_time);
  ApplicationReturnStatus status_ = ipopt_app_->Initialize();
  if (status_ != Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }

  // Convert the NLP problem to Ipopt
  IpoptPtr nlp_ptr = new IpoptAdapter(nlp_);
  status_ = ipopt_app_->OptimizeTNLP(nlp_ptr);
//  if (status_ == Solve_Succeeded) {
//    // Retrieve some statistics about the solve
//    Index iter_count = ipopt_app_->Statistics()->IterationCount();
//    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;
//
//    Number final_obj = ipopt_app_->Statistics()->FinalObjective();
//    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
//  }
//
//  if (status_ == Infeasible_Problem_Detected) {
//    std::cout << "Problem/Constraints infeasible; run again?";
//  }

  if (status_ != Solve_Succeeded) {
    std::string msg = "Ipopt failed to find a solution. ReturnCode: " + std::to_string(status_);
    throw std::runtime_error(msg);
  }
}



} /* namespace opt */
} /* namespace xpp */


