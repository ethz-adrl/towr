/**
 @file    nlp_facade.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Defines the class NlpFacade implementing the Facade Pattern.
 */

#include <xpp/opt/nlp_facade.h>

#include <xpp/opt/nlp.h>
#include <xpp/opt/constraint_container.h>
#include <xpp/opt/cost_constraint_factory.h>
#include <xpp/opt/cost_container.h>
#include <xpp/opt/motion_factory.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/opt/nlp.h>
#include <xpp/opt/optimization_variables.h>

#include <xpp/opt/ipopt_adapter.h>
#include <xpp/opt/snopt_adapter.h>

namespace xpp {
namespace opt {


NlpFacade::NlpFacade (VisualizerPtr visualizer)
     :visualizer_(visualizer)
{
  // create corresponding heap object for each of the member pointers
  opt_variables_ = std::make_shared<OptimizationVariables>();
  costs_         = std::make_shared<CostContainer>(*opt_variables_);
  constraints_   = std::make_shared<ConstraintContainer>(*opt_variables_);

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
                    const MotionStructure& motion_structure)
{
//  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases(), initial_state.p, initial_state.v);
  com_motion_ = MotionFactory::CreateComMotion(motion_structure.GetPhases());

  opt_variables_->ClearVariables();
  opt_variables_->AddVariableSet(CostConstraintFactory::CreateSplineCoeffVariables(*com_motion_));
  opt_variables_->AddVariableSet(CostConstraintFactory::CreateContactVariables(motion_structure, initial_state.p));
  opt_variables_->AddVariableSet(CostConstraintFactory::CreateConvexityVariables(motion_structure));
  opt_variables_->AddVariableSet(CostConstraintFactory::CreateCopVariables(motion_structure));


  constraints_->ClearConstraints();
  constraints_->AddConstraint(CostConstraintFactory::CreateInitialConstraint(initial_state, *com_motion_));
  constraints_->AddConstraint(CostConstraintFactory::CreateFinalConstraint(final_state, *com_motion_));
  constraints_->AddConstraint(CostConstraintFactory::CreateJunctionConstraint(*com_motion_));
  constraints_->AddConstraint(CostConstraintFactory::CreateDynamicConstraint(*com_motion_, motion_structure, robot_height));
  constraints_->AddConstraint(CostConstraintFactory::CreateSupportAreaConstraint(motion_structure));
  constraints_->AddConstraint(CostConstraintFactory::CreateConvexityContraint(motion_structure));
  constraints_->AddConstraint(CostConstraintFactory::CreateRangeOfMotionConstraint(*com_motion_, motion_structure));
//  constraints_->AddConstraint(CostConstraintFactory::CreateFinalStanceConstraint(final_state.p, motion_structure));


  costs_->ClearCosts();
  costs_->AddCost(CostConstraintFactory::CreateMotionCost(*com_motion_, utils::kAcc));
  costs_->AddCost(CostConstraintFactory::CreateRangeOfMotionCost(*com_motion_, motion_structure));
  costs_->AddCost(CostConstraintFactory::CreatePolygonCenterCost(motion_structure));

  // normalize the costs
  int n_nodes = motion_structure.GetPhaseStampedVec().size();
  int n_discrete_contacts = motion_structure.GetTotalNumberOfNodeContacts();
  int t_total = motion_structure.GetTotalTime();
  costs_->SetWeights({1.0/t_total, 30.0/n_discrete_contacts, 100.0/n_nodes});
//  costs_->SetWeights({1.0});


  visualizer_->SetOptimizationVariables(opt_variables_);
  visualizer_->SetComMotion(com_motion_);
  visualizer_->SetMotionStructure(motion_structure);

  std::unique_ptr<NLP> nlp(new NLP);
  nlp->Init(opt_variables_, costs_, constraints_);


//  // Snopt solving
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
NlpFacade::AttachNlpObserver (VisualizerPtr& visualizer)
{
  visualizer_ = visualizer;  // handle so ipopt can poll publish() method
}

NlpFacade::VecFoothold
NlpFacade::GetFootholds () const
{
  Eigen::VectorXd footholds = opt_variables_->GetVariables(VariableNames::kFootholds);
  return utils::ConvertEigToStd(footholds);
}

const NlpFacade::ComMotionPtrS
NlpFacade::GetComMotion() const
{
  Eigen::VectorXd x_motion = opt_variables_->GetVariables(VariableNames::kSplineCoeff);
  com_motion_->SetCoefficients(x_motion);
  return com_motion_;
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

} /* namespace opt */
} /* namespace xpp */

