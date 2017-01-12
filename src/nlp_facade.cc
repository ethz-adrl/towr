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

  nlp_ = std::make_shared<NLP>();
  nlp_->Init(opt_variables_, costs_, constraints_);

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
NlpFacade::AttachNlpObserver (VisualizerPtr& visualizer)
{
  visualizer_ = visualizer;  // handle so ipopt can poll publish() method
}

void
NlpFacade::BuildNlp(const State& initial_state,
                    const State& final_state,
                    double robot_height,
                    const MotionStructure& motion_structure,
                    const MotionparamsPtr& motion_params)
{
//  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases(), initial_state.p, initial_state.v);
  com_motion_ = MotionFactory::CreateComMotion(motion_structure.GetPhases(), motion_params->polynomials_per_phase_);

  CostConstraintFactory fac;
  fac.Init(com_motion_, motion_structure, motion_params);

  opt_variables_->ClearVariables();
  opt_variables_->AddVariableSet(fac.SplineCoeffVariables());
  opt_variables_->AddVariableSet(fac.ContactVariables(initial_state.p));
  opt_variables_->AddVariableSet(fac.ConvexityVariables());
  opt_variables_->AddVariableSet(fac.CopVariables());


  constraints_->ClearConstraints();
  constraints_->AddConstraint(fac.InitialConstraint_(initial_state));
  constraints_->AddConstraint(fac.FinalConstraint_(final_state));
  constraints_->AddConstraint(fac.FinalStanceConstraint_(final_state.p));
  constraints_->AddConstraint(fac.JunctionConstraint_());
  constraints_->AddConstraint(fac.DynamicConstraint_(robot_height));
  constraints_->AddConstraint(fac.SupportAreaConstraint_());
  constraints_->AddConstraint(fac.ConvexityConstraint_());
  constraints_->AddConstraint(fac.RangeOfMotionBoxConstraint_());

  costs_->ClearCosts();
  costs_->AddCost(fac.ComMotionCost_(utils::kAcc), motion_params->weight_com_motion_cost_);
  costs_->AddCost(fac.RangeOfMotionCost_(),        motion_params->weight_range_of_motion_cost_);
  costs_->AddCost(fac.PolygonCenterCost_(),        motion_params->weight_polygon_center_cost_);

//  int n_nodes = motion_structure.GetPhaseStampedVec().size();
//  int n_discrete_contacts = motion_structure.GetTotalNumberOfNodeContacts();
//  int t_total = motion_structure.GetTotalTime();
//  double weight_motion  =  1.0/t_total;
//  double weight_range   =  30.0/n_discrete_contacts;
//  double weight_polygon =  100.0/n_nodes;

  visualizer_->SetOptimizationVariables(opt_variables_);
  visualizer_->SetComMotion(com_motion_);
  visualizer_->SetMotionStructure(motion_structure);
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
  IpoptPtr nlp_ptr = new IpoptAdapter(nlp_, visualizer_); // just so it can poll the PublishMsg() method

  // some options to change on the fly
//  ipopt_solver_->Options()->SetNumericValue("max_cpu_time", max_cpu_time);
  status_ = ipopt_app_->Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }

  status_ = ipopt_app_->OptimizeTNLP(nlp_ptr);
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

