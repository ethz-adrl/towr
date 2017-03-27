/**
 @file    nlp_facade.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Defines the class NlpFacade implementing the Facade Pattern.
 */

#include <xpp/opt/nlp_facade.h>

#include <xpp/nlp.h>
#include <xpp/ipopt_adapter.h>
#include <xpp/snopt_adapter.h>

#include <xpp/opt/cost_constraint_factory.h>

namespace xpp {
namespace opt {


NlpFacade::NlpFacade ()
{
  // create corresponding heap object for each of the member pointers
  opt_variables_ = std::make_shared<OptimizationVariables>();
  costs_         = std::make_shared<CostContainer>(*opt_variables_);
  constraints_   = std::make_shared<ConstraintContainer>(*opt_variables_);

  nlp_ = std::make_shared<NLP>();
  nlp_->Init(opt_variables_, costs_, constraints_);
}

NlpFacade::~NlpFacade ()
{
}

void
NlpFacade::OptimizeMotion(const StateLin2d& initial_state,
                          const StateLin2d& final_state,
                          const EEMotionPtrS& ee_motion,
                          const ComMotionPtrS& com_motion,
                          const MotionparamsPtr& motion_params,
                          NlpSolver solver)
{
  // internal optimization variables
  auto ee_load = std::make_shared<EndeffectorLoad>();

  // zmp_ a this should work  also with a different value
  double parameter_dt = 0.05;//motion_params->dt_nodes_;
  ee_load->Init(*ee_motion, parameter_dt, ee_motion->GetTotalTime());

  auto cop = std::make_shared<CenterOfPressure>();
  cop->Init(parameter_dt, ee_motion->GetTotalTime());

  CostConstraintFactory factory;
  factory.Init(com_motion,
               ee_motion,
               ee_load,
               cop,
               motion_params,
               initial_state,
               final_state);

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


  SolveNlp(solver);

  Eigen::VectorXd xy = opt_variables_->GetVariables(EndeffectorsMotion::ID);
  ee_motion->SetOptimizationParameters(xy);

  Eigen::VectorXd x_motion = opt_variables_->GetVariables(ComMotion::ID);
  com_motion->SetCoefficients(x_motion);


//  int n_nodes = motion_structure.GetPhaseStampedVec().size();
//  int n_discrete_contacts = motion_structure.GetTotalNumberOfNodeContacts();
//  int t_total = motion_structure.GetTotalTime();
//  double weight_motion  =  1.0/t_total;
//  double weight_range   =  30.0/n_discrete_contacts;
//  double weight_polygon =  100.0/n_nodes;
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

//NlpFacade::ContactVec
//NlpFacade::GetContacts ()
//{
//  Eigen::VectorXd xy = opt_variables_->GetVariables(VariableNames::kFootholds);
//
//  int k=0;
//  for (int i=0; i<contacts_.size(); ++i) {
//    contacts_.at(i).p.x() = xy(k++);
//    contacts_.at(i).p.y() = xy(k++);
//    contacts_.at(i).p.z() = 0.0;
//  }
//
//  return contacts_;
//}

} /* namespace opt */
} /* namespace xpp */

