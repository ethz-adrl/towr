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
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/constraint_factory.h>
#include <xpp/zmp/cost_factory.h>
#include <xpp/zmp/optimization_variables_interpreter.h>
#include <xpp/zmp/interpreting_observer.h>

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



  // initialize the ipopt solver
  ipopt_solver_.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  status_ = ipopt_solver_.Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }
}

void
NlpFacade::InitializeVariables (int n_spline_coeff, int n_footholds)
{
  opt_variables_->Init(n_spline_coeff, n_footholds);
}

void
NlpFacade::InitializeVariables (const Eigen::VectorXd& spline_abcd_coeff,
                                const StdVecEigen2d& footholds)
{
  opt_variables_->Init(spline_abcd_coeff, footholds);
}

void
NlpFacade::SolveNlp(const Eigen::Vector2d& initial_acc,
                    const State& final_state,
                    const InterpreterPtr& interpreter_ptr)
{
  // save the framework of the optimization problem
  interpreting_observer_->SetInterpreter(interpreter_ptr);

  ContinuousSplineContainer spline_structure = interpreter_ptr->GetSplineStructure();

  constraints_->ClearConstraints();
  constraints_->AddConstraint(ConstraintFactory::CreateAccConstraint(initial_acc, spline_structure.GetTotalFreeCoeff()), "acc");
  constraints_->AddConstraint(ConstraintFactory::CreateFinalConstraint(final_state, spline_structure), "final");
  constraints_->AddConstraint(ConstraintFactory::CreateJunctionConstraint(spline_structure), "junction");
  constraints_->AddConstraint(ConstraintFactory::CreateZmpConstraint(*interpreter_ptr), "zmp");
  constraints_->AddConstraint(ConstraintFactory::CreateRangeOfMotionConstraint(*interpreter_ptr), "rom");
//  constraints_->AddConstraint(ConstraintFactory::CreateJointAngleConstraint(*interpreter_ptr), "joint_angles");

  costs_->ClearCosts();
  costs_->AddCost(CostFactory::CreateAccelerationCost(spline_structure), "cost_acc");
  // careful: these are not quite debugged yet
//  costs_->AddCost(CostFactory::CreateFinalCost(final_state, spline_structure), "cost_final");
//  costs_->AddCost(CostFactory::CreateRangeOfMotionCost(*interpreter_ptr), "cost_rom");

  std::unique_ptr<NLP> nlp(new NLP);
  nlp->Init(opt_variables_, costs_, constraints_);

  // Snopt solving
//  auto snopt_problem = SnoptAdapter::GetInstance();
//  snopt_problem->SetNLP(nlp);
//  snopt_problem->Init();
//  int Cold = 0, Basis = 1, Warm = 2;
//  snopt_problem->solve(Cold);
//  opt_variables_->SetVariables(snopt_problem->GetVariables());

  // Ipopt solving
  IpoptPtr nlp_ptr = new IpoptAdapter(*nlp, *visualizer_); // just so it can poll the PublishMsg() method
  SolveIpopt(nlp_ptr);
}

void
NlpFacade::SolveIpopt (const IpoptPtr& nlp)
{
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

