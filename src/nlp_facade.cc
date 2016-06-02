/*
 * nlp_optimizer.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/nlp_facade.h>

#include <xpp/zmp/spline_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/initial_acceleration_equation.h>
#include <xpp/zmp/final_state_equation.h>
#include <xpp/zmp/spline_junction_equation.h>
#include <xpp/zmp/ipopt_adapter.h>

namespace xpp {
namespace zmp {

NlpFacade::NlpFacade (AObserverVisualizer& visualizer)
    :c_acc_(opt_variables_),
     c_final_(opt_variables_),
     c_junction_(opt_variables_),
     c_zmp_(opt_variables_),
     c_rom_(opt_variables_),
     cost_container_(opt_variables_),
     cost_acc_(opt_variables_),
     cost_rom_(opt_variables_),
     visualizer_(&visualizer)
{
  constraints_.AddConstraint(c_acc_, "acc");
  constraints_.AddConstraint(c_final_, "final");
  constraints_.AddConstraint(c_junction_, "junction");
  constraints_.AddConstraint(c_zmp_, "zmp");
  constraints_.AddConstraint(c_rom_, "rom");

  cost_container_.AddCost(cost_acc_);
  cost_container_.AddCost(cost_rom_);

  // initialize the ipopt solver
  ipopt_solver_.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  status_ = ipopt_solver_.Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }
}

void
NlpFacade::AttachVisualizer (AObserverVisualizer& visualizer)
{
  visualizer_ = &visualizer;
  visualizer_->RegisterWithSubject(opt_variables_);
}

void
NlpFacade::SolveNlp(const State& initial_state,
                       const State& final_state,
                       const std::vector<xpp::hyq::LegID>& step_sequence,
                       const VecFoothold& start_stance,
                       const SplineTimes& times,
                       double robot_height)
{
  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance, step_sequence, xpp::hyq::SupportPolygon::GetDefaultMargins());

  opt_variables_.Init(initial_state.p, initial_state.v, step_sequence, times);
  opt_variables_.SetFootholds(supp_polygon_container.GetFootholdsInitializedToStart());

  ContinuousSplineContainer spline_structure;
  spline_structure.Init(initial_state.p, initial_state.v, step_sequence, times);

  // This should all be hidden inside a factory method
  // the linear equations
  InitialAccelerationEquation eq_acc(initial_state.a, spline_structure.GetTotalFreeCoeff());
  FinalStateEquation eq_final(final_state, spline_structure);
  SplineJunctionEquation eq_junction(spline_structure);

  // initialize the constraints
  dynamic_cast<LinearEqualityConstraint&>(constraints_.GetConstraint("acc")).Init(eq_acc.BuildLinearEquation());
  dynamic_cast<LinearEqualityConstraint&>(constraints_.GetConstraint("final")).Init(eq_final.BuildLinearEquation());
  dynamic_cast<LinearEqualityConstraint&>(constraints_.GetConstraint("junction")).Init(eq_junction.BuildLinearEquation());
  dynamic_cast<ZmpConstraint&>(constraints_.GetConstraint("zmp")).Init(spline_structure, supp_polygon_container, robot_height);
  dynamic_cast<RangeOfMotionConstraint&>(constraints_.GetConstraint("rom")).Init(spline_structure, supp_polygon_container);
  constraints_.Refresh();

  // costs
  TotalAccelerationEquation eq_total_acc(spline_structure);
  cost_acc_.Init(eq_total_acc.BuildLinearEquation());
  cost_rom_.Init(spline_structure, supp_polygon_container);

  // todo create complete class out of these input arguments
  IpoptPtr nlp_ptr = new Ipopt::IpoptAdapter(opt_variables_,
                                             cost_container_,
                                             constraints_,
                                             *visualizer_);
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

NlpFacade::VecFoothold
NlpFacade::GetFootholds () const
{
  return opt_variables_.GetFootholds();
}

NlpFacade::VecSpline
NlpFacade::GetSplines ()
{
  return opt_variables_.GetSplines();
}

} /* namespace zmp */
} /* namespace xpp */

