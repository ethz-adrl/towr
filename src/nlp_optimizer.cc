/*
 * nlp_optimizer.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/nlp_optimizer.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/spline_constraints.h>


#include <xpp/zmp/optimization_variables.h>

#include <xpp/zmp/initial_acceleration_constraint.h>
#include <xpp/zmp/final_state_constraint.h>
#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/range_of_motion_constraint.h>

#include <xpp/zmp/constraint_container.h>

namespace xpp {
namespace zmp {


NlpOptimizer::NlpOptimizer (IVisualizer& visualizer)
    :visualizer_(visualizer)
{
  app_.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  status_ = app_.Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }
}


void
NlpOptimizer::SolveNlp(const State& initial_state,
                       const State& final_state,
                       const std::vector<xpp::hyq::LegID>& step_sequence,
                       const VecFoothold& start_stance,
                       const SplineTimes& times,
                       double robot_height,
                       VecSpline& opt_splines,
                       VecFoothold& opt_footholds)
{
  typedef xpp::hyq::SupportPolygon SupportPolygon;

  // create the general spline structure
  ContinuousSplineContainer spline_structure;
  spline_structure.Init(initial_state.p,
                        initial_state.v ,
                        step_sequence,
                        times);


  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance,
                              step_sequence,
                              SupportPolygon::GetDefaultMargins());

  NlpStructure nlp_structure(spline_structure.GetTotalFreeCoeff(),
                             supp_polygon_container.GetNumberOfSteps());


  // fixme this should also change if the goal or any other parameter changes apart from the start position
  bool init_with_zeros = true;
  bool num_steps_changed = initial_variables_.footholds_.size() != nlp_structure.n_steps_;
  if (num_steps_changed || init_with_zeros) {
    SetInitialVariables(nlp_structure, supp_polygon_container);
  } else {} // use previous values


//  Constraints constraints(supp_polygon_container, spline_structure, nlp_structure, robot_height, initial_state.a, final_state);
  CostFunction cost_function(spline_structure, supp_polygon_container, nlp_structure);



  // new stuff

  OptimizationVariables subject(spline_structure.GetTotalFreeCoeff(), supp_polygon_container.GetNumberOfSteps());
  InitialAccelerationConstraint c1(subject);
  c1.Init(initial_state.a);

  State des_final = final_state;
  des_final.p.x() = -0.1;

  FinalStateConstraint c2(subject);
  c2.Init(final_state, spline_structure);

  ZmpConstraint c3(subject);
  c3.Init(spline_structure, supp_polygon_container, robot_height);

  RangeOfMotionConstraint c4(subject);
  c4.Init(spline_structure, supp_polygon_container);




  ConstraintContainer constraint_container;
  constraint_container.AddConstraint(c1);
  constraint_container.AddConstraint(c2);
  constraint_container.AddConstraint(c3);
  constraint_container.AddConstraint(c4);

  // end of observer pattern stuff



  Ipopt::SmartPtr<Ipopt::NlpIpoptZmp> nlp_ipopt_zmp =
      new Ipopt::NlpIpoptZmp(cost_function,
                             subject, // optmization variables
                             constraint_container,
                             nlp_structure,
                             visualizer_,
                             initial_variables_);

  status_ = app_.OptimizeTNLP(nlp_ipopt_zmp);
  if (status_ == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app_.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app_.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
  }


  // save the solution for initialization of the next loop
  initial_variables_ = nlp_ipopt_zmp->opt_variables_;


  int n_steps = initial_variables_.footholds_.size();
  opt_footholds.resize(n_steps);
  for (int i=0; i<n_steps; ++i) {
    opt_footholds.at(i).leg = step_sequence.at(i);
  }

  xpp::hyq::Foothold::SetXy(initial_variables_.footholds_, opt_footholds);

  spline_structure.AddOptimizedCoefficients(initial_variables_.spline_coeff_);
  opt_splines = spline_structure.GetSplines();
}



void
NlpOptimizer::SetInitialVariables(const NlpStructure& nlp_structure,
                                  const SupportPolygonContainer& supp_polygons)
{
  initial_variables_ = NlpStructure::NlpVariables(nlp_structure);
  initial_variables_.spline_coeff_.setZero();
  initial_variables_.footholds_ = supp_polygons.GetFootholdsInitializedToStart();
}


} /* namespace zmp */
} /* namespace xpp */
