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

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

namespace xpp {
namespace opt {


NlpFacade::NlpFacade (VisualizerPtr visualizer)
     :visualizer_(visualizer)
{
  // create corresponding heap object for each of the member pointers
  opt_variables_ = std::make_shared<OptimizationVariables>();
  costs_         = std::make_shared<CostContainer>(*opt_variables_);
  constraints_   = std::make_shared<ConstraintContainer>(*opt_variables_);

  nlp_ = std::make_shared<NLP>();
  nlp_->Init(opt_variables_, costs_, constraints_);
}

void
NlpFacade::SetVisualizer (VisualizerPtr& visualizer)
{
  visualizer_ = visualizer;  // handle so ipopt can poll publish() method
}

void
NlpFacade::BuildNlp(const State& initial_state,
                    const State& final_state,
                    const MotionStructure& motion_structure,
                    const MotionparamsPtr& motion_params)
{

//  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetTotalTime(),
//                                                   motion_params->polynomials_per_second_,
//                                                   initial_state.p, initial_state.v);
  com_motion_ = MotionFactory::CreateComMotion(motion_structure,
                                               motion_params->polynomials_per_second_,
                                               motion_params->geom_walking_height_ + motion_params->offset_geom_to_com_.z());

  CostConstraintFactory factory;
  factory.Init(com_motion_, motion_structure, motion_params, initial_state, final_state);

  opt_variables_->ClearVariables();
  opt_variables_->AddVariableSet(factory.SplineCoeffVariables());
  opt_variables_->AddVariableSet(factory.ContactVariables(initial_state.p));
  opt_variables_->AddVariableSet(factory.ConvexityVariables());
  opt_variables_->AddVariableSet(factory.CopVariables());


  constraints_->ClearConstraints();
  for (ConstraintName name : motion_params->GetUsedConstraints()) {
    constraints_->AddConstraint(factory.GetConstraint(name));
  }

  costs_->ClearCosts();
  for (const auto& pair : motion_params->GetCostWeights()) {
    CostName name = pair.first;
    double weight = pair.second;
    costs_->AddCost(factory.GetCost(name), weight);
  }

//  int n_nodes = motion_structure.GetPhaseStampedVec().size();
//  int n_discrete_contacts = motion_structure.GetTotalNumberOfNodeContacts();
//  int t_total = motion_structure.GetTotalTime();
//  double weight_motion  =  1.0/t_total;
//  double weight_range   =  30.0/n_discrete_contacts;
//  double weight_polygon =  100.0/n_nodes;

  visualizer_->SetOptimizationVariables(opt_variables_);
  visualizer_->SetComMotion(com_motion_);
  visualizer_->SetMotionStructure(motion_structure);
  visualizer_->SetMotionParameters(motion_params);
}

void
NlpFacade::SolveNlp (NlpSolver solver)
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
NlpFacade::VisualizeSolution () const
{
  visualizer_->Visualize();
}

void
NlpFacade::SolveSnopt ()
{
  auto snopt_problem = SnoptAdapter::GetInstance();
  snopt_problem->SetNLP(nlp_);
  snopt_problem->Init();
  int Cold = 0, Basis = 1, Warm = 2;
  snopt_problem->SolveSQP(Cold);
}

void
NlpFacade::SolveIpopt ()
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
  IpoptPtr nlp_ptr = new IpoptAdapter(nlp_, visualizer_); // just so it can poll the PublishMsg() method
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

} /* namespace opt */
} /* namespace xpp */

