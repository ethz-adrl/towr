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
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include <xpp/zmp/initial_acceleration_equation.h>
#include <xpp/zmp/final_state_equation.h>
#include <xpp/zmp/spline_junction_equation.h>

namespace xpp {
namespace zmp {

NlpFacade::NlpFacade (AObserverVisualizer& visualizer)
    :c_acc_(subject_),
     c_final_(subject_),
     c_junction_(subject_),
     c_zmp_(subject_),
     c_rom_(subject_),
     cost_container_(subject_),
     cost_acc_(subject_),
     cost_rom_(subject_),
     visualizer_(&visualizer)
{

  app_.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  status_ = app_.Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }


  constraints_.AddConstraint(c_acc_);
  constraints_.AddConstraint(c_final_);
  constraints_.AddConstraint(c_junction_);
  constraints_.AddConstraint(c_zmp_);
  constraints_.AddConstraint(c_rom_);

  cost_container_.AddCost(cost_acc_);
  cost_container_.AddCost(cost_rom_);
}

void
NlpFacade::AttachVisualizer (AObserverVisualizer& visualizer)
{
  visualizer_ = &visualizer;
  visualizer_->RegisterWithSubject(subject_);
}

void
NlpFacade::SolveNlp(const State& initial_state,
                       const State& final_state,
                       const std::vector<xpp::hyq::LegID>& step_sequence,
                       const VecFoothold& start_stance,
                       const SplineTimes& times,
                       double robot_height)
{
  typedef xpp::hyq::SupportPolygon SupportPolygon;

  // create the general spline structure
  // fixme, can probably remove this
  ContinuousSplineContainer spline_structure;
  spline_structure.Init(initial_state.p,
                        initial_state.v ,
                        step_sequence,
                        times);

  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance,
                              step_sequence,
                              SupportPolygon::GetDefaultMargins());

  subject_.Init(initial_state.p,
                initial_state.v,
                step_sequence,
                times);

  subject_.SetFootholds(supp_polygon_container.GetFootholdsInitializedToStart());


//  NlpStructure nlp_structure(spline_structure_.GetTotalFreeCoeff(),
//                             supp_polygon_container.GetNumberOfSteps());

  // fixme this should also change if the goal or any other parameter changes apart from the start position
//  bool init_with_zeros = true;
//  bool num_steps_changed = initial_variables_.footholds_.size() != nlp_structure.n_steps_;
//  if (num_steps_changed || init_with_zeros) {
//    SetInitialVariables(nlp_structure, supp_polygon_container);
//  } else {} // use previous values

  // add the constraints
  // This should all be hidden inside a factory method
  // the linear equations
  InitialAccelerationEquation eq_acc(initial_state.a, spline_structure.GetTotalFreeCoeff());
  FinalStateEquation eq_final(final_state, spline_structure);
  SplineJunctionEquation eq_junction(spline_structure);

  c_acc_.Init(eq_acc.BuildLinearEquation());
  c_final_.Init(eq_final.BuildLinearEquation());
  c_zmp_.Init(spline_structure, supp_polygon_container, robot_height);
  c_rom_.Init(spline_structure, supp_polygon_container);
  c_junction_.Init(eq_junction.BuildLinearEquation());
  constraints_.Refresh();

  // costs
  TotalAccelerationEquation eq_total_acc(spline_structure);
  cost_acc_.Init(eq_total_acc.BuildLinearEquation());
  cost_rom_.Init(spline_structure, supp_polygon_container);


  // todo make this class ipoptAdapter
  IpoptPtr nlp_ptr = new Ipopt::NlpIpoptZmp(subject_, // optmization variables
                                            cost_container_,
                                            constraints_,
                                            *visualizer_);

  SolveIpopt(nlp_ptr);
}

void
NlpFacade::SolveIpopt (const IpoptPtr& nlp)
{
  status_ = app_.OptimizeTNLP(nlp);
  if (status_ == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app_.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app_.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
  }
}

NlpFacade::VecFoothold
NlpFacade::GetFootholds () const
{
  return subject_.GetFootholds();
}

NlpFacade::VecSpline
NlpFacade::GetSplines ()
{
  return subject_.GetSplines();
}

} /* namespace zmp */
} /* namespace xpp */

