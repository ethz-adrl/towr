/**
 @file    nlp_facade.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Defines the class NlpFacade implementing the Facade Pattern.
 */

#include <xpp/opt/nlp_facade.h>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/opt/nlp.h>
#include <xpp/opt/constraint_container.h>
#include <xpp/opt/cost_constraint_factory.h>
#include <xpp/opt/cost_container.h>
#include <xpp/opt/ipopt_adapter.h>
#include <xpp/opt/motion_factory.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/opt/nlp.h>
#include <xpp/opt/nlp_observer.h>
#include <xpp/opt/optimization_variables.h>

#include <xpp/hyq/hyq_robot_interface.h>
//#include <xpp/opt/snopt_adapter.h>

#include <iomanip>

namespace xpp {
namespace opt {

using Contacts = xpp::hyq::SupportPolygonContainer;

NlpFacade::NlpFacade (VisualizerPtr visualizer)
     :visualizer_(visualizer)
{
  // create corresponding heap object for each of the member pointers
  opt_variables_ = std::make_shared<OptimizationVariables>();
  costs_         = std::make_shared<CostContainer>(*opt_variables_);
  constraints_   = std::make_shared<ConstraintContainer>(*opt_variables_);
  nlp_observer_  = std::make_shared<NlpObserver>(*opt_variables_);


  // initialize the ipopt solver
  ipopt_app_ = new Ipopt::IpoptApplication();
  ipopt_app_->RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt


  status_ = ipopt_app_->Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }
}

void
NlpFacade::SolveNlp(const State& initial_state,
                    const State& final_state,
                    double robot_height,
                    const MotionStructure& motion_structure,
                    const Contacts& contacts,
                    double dt_zmp)
{
  // insight: this spline might be better for model pred. control, as it always matches the initial
  // position and velocity, avoiding jumps in state. For the other spline this is
  // a constraint, that might not be fulfilled.
  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases(), initial_state.p, initial_state.v);
//  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases());

  nlp_observer_->Init(motion_structure, *com_motion, contacts);

  // provide the initial values of the optimization problem
  opt_variables_->ClearVariables();
  opt_variables_->AddVariableSet(VariableNames::kSplineCoeff, com_motion->GetCoeffients());
  opt_variables_->AddVariableSet(VariableNames::kFootholds,
                                 contacts.GetFootholdsInitializedToNominal(initial_state.p),
                                 AConstraint::Bound(-10, 10));
  Eigen::VectorXd lambdas(motion_structure.GetTotalNumberOfNodeContacts());
  lambdas.fill(0.33);
  opt_variables_->AddVariableSet(VariableNames::kConvexity, lambdas, AConstraint::Bound(0.12, 1.0));

  // center of pressure x,y (input) at every node (changes system dynamics)
  int n = motion_structure.GetPhaseStampedVec().size() * utils::kDim2d;
  Eigen::VectorXd cop(n);
  cop.setZero();
  opt_variables_->AddVariableSet(VariableNames::kCenterOfPressure, cop, AConstraint::Bound(-10, 10));





  constraints_->ClearConstraints();
  constraints_->AddConstraint(CostConstraintFactory::CreateInitialConstraint(initial_state, *com_motion));
  constraints_->AddConstraint(CostConstraintFactory::CreateFinalConstraint(final_state, *com_motion));
  constraints_->AddConstraint(CostConstraintFactory::CreateJunctionConstraint(*com_motion));


//  constraints_->AddConstraint(CostConstraintFactory::CreateZmpConstraint(motion_structure,
//                                                                         *com_motion,
//                                                                         contacts,
//                                                                         robot_height,
//                                                                         dt_zmp));

  constraints_->AddConstraint(CostConstraintFactory::CreateDynamicConstraint(motion_structure,
                                                                             *com_motion,
                                                                             contacts));
  constraints_->AddConstraint(CostConstraintFactory::CreateConvexityContraint(motion_structure));



  constraints_->AddConstraint(CostConstraintFactory::CreateRangeOfMotionConstraint(*com_motion, contacts, motion_structure));
  constraints_->AddConstraint(CostConstraintFactory::CreateFinalStanceConstraint(final_state.p, contacts));

   // careful: these are not quite debugged yet
//  constraints_->AddConstraint(CostConstraintFactory::CreateObstacleConstraint(contacts));
//  constraints_->AddConstraint(ConstraintFactory::CreateJointAngleConstraint(*interpreter_ptr));

  costs_->ClearCosts();
  costs_->AddCost(CostConstraintFactory::CreateMotionCost(*com_motion, utils::kAcc));
  costs_->AddCost(CostConstraintFactory::CreateRangeOfMotionCost(*com_motion, contacts, motion_structure));
//  costs_->AddCost(CostConstraintFactory::CreateFinalStanceCost(final_state.p, contacts));
  costs_->SetWeights({1.0, 1.0});
  // careful: these are not quite debugged yet
//  costs_->AddCost(CostConstraintFactory::CreateFinalComCost(final_state, spline_structure));
//  costs_->AddCost(CostConstraintFactory::CreateRangeOfMotionCost(interpreter));




  std::cout << std::setprecision(2) << std::fixed;
  std::cout << "start_state: " << initial_state << std::endl;
  std::cout << "goal_state: " << final_state << std::endl;

  std::cout << "\nstart_stance: \n";
  for (auto f : contacts.GetStartStance())
    std::cout << f << std::endl;

  std::cout << "\nphases:\n";
  for (auto phase : motion_structure.GetPhases()) {
    std::cout << phase << std::endl;
  }

//  std::cout << "\npolynomials:\n";
//  auto com_spline = std::dynamic_pointer_cast<ComSpline> (com_motion);
//  for (auto poly : com_spline->GetPolynomials()) {
//    std::cout << poly << std::endl;
//  }



//  NLP nlp;
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
  IpoptPtr nlp_ptr = new IpoptAdapter(*nlp, visualizer_); // just so it can poll the PublishMsg() method
  SolveIpopt(nlp_ptr);
}

void
NlpFacade::SolveIpopt (const IpoptPtr& nlp)
{
  // some options to change on the fly
//  ipopt_solver_->Options()->SetNumericValue("max_cpu_time", max_cpu_time);
  status_ = ipopt_app_->Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }

  status_ = ipopt_app_->OptimizeTNLP(nlp);
  if (status_ == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = ipopt_app_->Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = ipopt_app_->Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
  }

  if (status_ == Ipopt::Infeasible_Problem_Detected) {
    std::cout << "Problem/Constraints infeasible; run again?";
  }
}

void
NlpFacade::AttachNlpObserver (VisualizerPtr& visualizer)
{
  visualizer->SetObserver(nlp_observer_); // current values of optimization variables
  visualizer_ = visualizer;               // handle so ipopt can poll publish() method
}

NlpFacade::VecFoothold
NlpFacade::GetFootholds () const
{
  return nlp_observer_->GetFootholds();
}

NlpFacade::ComMotionPtrS
NlpFacade::GetMotion () const
{
  return nlp_observer_->GetComMotion();
}

PhaseVec
NlpFacade::GetPhases () const
{
  return nlp_observer_->GetStructure().GetPhases();
}

} /* namespace zmp */
} /* namespace xpp */

