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

using Fac = CostConstraintFactory;

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
                    const MotionStructure& ms,
                    const MotionTypePtr& motion_type)
{
//  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases(), initial_state.p, initial_state.v);
  com_motion_ = MotionFactory::CreateComMotion(ms.GetPhases());

  opt_variables_->ClearVariables();
  opt_variables_->AddVariableSet(Fac::SplineCoeffVariables(*com_motion_));
  opt_variables_->AddVariableSet(Fac::ContactVariables(ms, initial_state.p));
  opt_variables_->AddVariableSet(Fac::ConvexityVariables(ms));
  opt_variables_->AddVariableSet(Fac::CopVariables(ms));


  constraints_->ClearConstraints();
  constraints_->AddConstraint(Fac::InitialConstraint_(initial_state, *com_motion_));
  constraints_->AddConstraint(Fac::FinalConstraint_(final_state, *com_motion_));
  constraints_->AddConstraint(Fac::JunctionConstraint_(*com_motion_));
  constraints_->AddConstraint(Fac::DynamicConstraint_(*com_motion_, ms, robot_height));
  constraints_->AddConstraint(Fac::SupportAreaConstraint_(ms));
  constraints_->AddConstraint(Fac::ConvexityConstraint_(ms));
  constraints_->AddConstraint(Fac::RangeOfMotionBoxConstraint_(*com_motion_, ms));
  constraints_->AddConstraint(Fac::FinalStanceConstraint_(final_state.p, ms));

  costs_->ClearCosts();
  costs_->AddCost(Fac::ComMotionCost_(*com_motion_, utils::kAcc),motion_type->weight_com_motion_cost_);
  costs_->AddCost(Fac::RangeOfMotionCost_(*com_motion_, ms),     motion_type->weight_range_of_motion_cost_);
  costs_->AddCost(Fac::PolygonCenterCost_(ms),                   motion_type->weight_polygon_center_cost_);

//  int n_nodes = motion_structure.GetPhaseStampedVec().size();
//  int n_discrete_contacts = motion_structure.GetTotalNumberOfNodeContacts();
//  int t_total = motion_structure.GetTotalTime();
//  double weight_motion  =  1.0/t_total;
//  double weight_range   =  30.0/n_discrete_contacts;
//  double weight_polygon =  100.0/n_nodes;


  visualizer_->SetOptimizationVariables(opt_variables_);
  visualizer_->SetComMotion(com_motion_);
  visualizer_->SetMotionStructure(ms);

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

