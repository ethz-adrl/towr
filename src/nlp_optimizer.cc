/*
 * nlp_optimizer.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/nlp_optimizer.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/spline_constraints.h>

namespace xpp {
namespace zmp {


NlpOptimizer::NlpOptimizer ()
    :zmp_publisher_("nlp_zmp_publisher")
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







  NlpStructure::NlpVariables initial_variables(nlp_structure);
  // initialize with start stance
  for (uint step=0; step<initial_variables.footholds_.size(); ++step) {
    xpp::hyq::LegID leg = supp_polygon_container.GetLegID(step);
    initial_variables.footholds_.at(step) = supp_polygon_container.GetStartFoothold(leg).GetXy();
  }





  Constraints constraints(supp_polygon_container,
                          spline_structure,
                          nlp_structure,
                          robot_height,
                          initial_state.a,
                          final_state);
  CostFunction cost_function(spline_structure, supp_polygon_container, nlp_structure);


  Ipopt::SmartPtr<Ipopt::NlpIpoptZmp> nlp_ipopt_zmp =
      new Ipopt::NlpIpoptZmp(cost_function,
                             constraints,
                             nlp_structure,
                             zmp_publisher_,
                             initial_variables);

  status_ = app_.OptimizeTNLP(nlp_ipopt_zmp);
  if (status_ == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app_.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app_.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
  }


  // build the output for the user from the optimized variables
  NlpStructure::NlpVariables optimized_variables = nlp_ipopt_zmp->opt_variables_;


  int n_steps = optimized_variables.footholds_.size();
  opt_footholds.resize(n_steps);
  for (int i=0; i<n_steps; ++i) {
    opt_footholds.at(i).leg = step_sequence.at(i);
  }

  xpp::hyq::Foothold::SetXy(optimized_variables.footholds_, opt_footholds);


  spline_structure.AddOptimizedCoefficients(optimized_variables.spline_coeff_);
  opt_splines = spline_structure.GetSplines();
}




} /* namespace zmp */
} /* namespace xpp */
