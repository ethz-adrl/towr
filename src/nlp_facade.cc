/*
 * nlp_optimizer.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/nlp_facade.h>

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/cost_container.h>
#include <xpp/zmp/optimization_variables_interpreter.h>
#include <xpp/zmp/interpreting_observer.h>
#include <xpp/hyq/step_sequence_planner.h>
#include <xpp/zmp/cost_constraint_factory.h>
#include <xpp/zmp/motion_factory.h>

#include <xpp/zmp/nlp.h>
#include <xpp/zmp/ipopt_adapter.h>
#include <xpp/zmp/snopt_adapter.h>

namespace xpp {
namespace zmp {

NlpFacade::NlpFacade (IVisualizer& visualizer)
     :visualizer_(&visualizer)
{
  // create corresponding heap object for each of the member pointers
  opt_variables_         = std::make_shared<OptimizationVariables>();
  costs_                 = std::make_shared<CostContainer>(*opt_variables_);
  constraints_           = std::make_shared<ConstraintContainer>(*opt_variables_);
  interpreting_observer_ = std::make_shared<InterpretingObserver>(*opt_variables_);
  step_sequence_planner_ = std::make_shared<xpp::hyq::StepSequencePlanner>();


  // initialize the ipopt solver
  ipopt_solver_.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt


  status_ = ipopt_solver_.Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }
}

void
NlpFacade::SolveNlp(const State& initial_state,
                    const State& final_state,
                    int curr_swing_leg,
                    double robot_height,
                    VecFoothold curr_stance,
                    xpp::hyq::MarginValues margins,
                    xpp::zmp::SplineTimes spline_times_,
                    double max_cpu_time)
{
  step_sequence_planner_->Init(initial_state, final_state, curr_stance, robot_height);
  std::vector<xpp::hyq::LegID> step_sequence = step_sequence_planner_->DetermineStepSequence(curr_swing_leg);
  bool start_with_com_shift = step_sequence_planner_->StartWithStancePhase(step_sequence);

  std::cout << "start_with_com_shift: " << start_with_com_shift;

  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(curr_stance, step_sequence, margins);

  auto com_spline = MotionFactory::CreateComMotion(initial_state.p, initial_state.v, step_sequence.size(), spline_times_, start_with_com_shift);
//  auto com_spline = MotionFactory::CreateComMotion(step_sequence.size(), spline_times_, start_with_com_shift);

  opt_variables_->AddVariableSet(VariableNames::kSplineCoeff, com_spline->GetCoeffients());
  opt_variables_->AddVariableSet(VariableNames::kFootholds, supp_polygon_container.GetFootholdsInitializedToStart());

  // save the framework of the optimization problem
  OptimizationVariablesInterpreter interpreter;
  interpreter.Init(com_spline, supp_polygon_container, robot_height);
  interpreting_observer_->SetInterpreter(interpreter);

  constraints_->ClearConstraints();
  constraints_->AddConstraint(CostConstraintFactory::CreateInitialConstraint(initial_state, com_spline));
  constraints_->AddConstraint(CostConstraintFactory::CreateFinalConstraint(final_state, com_spline));
  constraints_->AddConstraint(CostConstraintFactory::CreateJunctionConstraint(com_spline));
  constraints_->AddConstraint(CostConstraintFactory::CreateZmpConstraint(interpreter));
  constraints_->AddConstraint(CostConstraintFactory::CreateRangeOfMotionConstraint(interpreter));
//  constraints_->AddConstraint(ConstraintFactory::CreateObstacleConstraint());
//  constraints_->AddConstraint(ConstraintFactory::CreateJointAngleConstraint(*interpreter_ptr));

  costs_->ClearCosts();
  costs_->AddCost(CostConstraintFactory::CreateAccelerationCost(com_spline));
  // careful: these are not quite debugged yet
//  costs_->AddCost(CostConstraintFactory::CreateFinalStanceCost(final_state.p, supp_polygon_container));
//  costs_->AddCost(CostConstraintFactory::CreateFinalComCost(final_state, spline_structure));
//  costs_->AddCost(CostConstraintFactory::CreateRangeOfMotionCost(interpreter));

  std::unique_ptr<NLP> nlp(new NLP);
  nlp->Init(opt_variables_, costs_, constraints_);

//  // Snopt solving
//  // fixme some constraints are still linear and have constant terms in its
//  // values, so this wont work
//  auto snopt_problem = SnoptAdapter::GetInstance();
//  snopt_problem->SetNLP(nlp);
//  snopt_problem->Init();
//  int Cold = 0, Basis = 1, Warm = 2;
//  snopt_problem->SolveSQP(Cold);

  // Ipopt solving
  IpoptPtr nlp_ptr = new IpoptAdapter(*nlp, *visualizer_); // just so it can poll the PublishMsg() method
  SolveIpopt(nlp_ptr, max_cpu_time);
}

void
NlpFacade::SolveIpopt (const IpoptPtr& nlp, double max_cpu_time)
{
  // some options to change on the fly
  ipopt_solver_.Options()->SetNumericValue("max_cpu_time", max_cpu_time);

  status_ = ipopt_solver_.OptimizeTNLP(nlp);
  if (status_ == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = ipopt_solver_.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = ipopt_solver_.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
  }
}

NlpFacade::InterpretingObserverPtr
NlpFacade::GetObserver () const
{
  return interpreting_observer_;
}

void
NlpFacade::AttachVisualizer (IVisualizer& visualizer)
{
  visualizer_ = &visualizer;
}

NlpFacade::VecFoothold
NlpFacade::GetFootholds () const
{
  return interpreting_observer_->GetFootholds();
}

NlpFacade::VecSpline
NlpFacade::GetSplines () const
{
  return interpreting_observer_->GetSplines();
}

} /* namespace zmp */
} /* namespace xpp */

