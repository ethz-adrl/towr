/*
 * nlp_optimizer.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/nlp_optimizer.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>

#include <xpp/zmp/nlp_ipopt_zmp.h>
#include <xpp/ros/ros_helpers.h>

namespace xpp {
namespace zmp {


void
NlpOptimizer::SolveNlp(const State& initial_state,
                       const State& final_state,
                       const std::vector<xpp::hyq::LegID>& step_sequence,
                       xpp::hyq::LegDataMap<Foothold> start_stance,
                       VecSpline& opt_splines,
                       StdVecEigen2d& final_footholds,
                       const Eigen::VectorXd& initial_spline_coeff) const
{
  Ipopt::IpoptApplication app;
  app.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  Ipopt::ApplicationReturnStatus status = app.Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }



  // create the general spline structure
  ContinuousSplineContainer spline_structure;
  double swing_time          = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  double stance_time         = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time");
  double stance_time_initial = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");
  double stance_time_final   = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time_final");
  spline_structure.Init(initial_state.p, initial_state.v ,step_sequence, stance_time, swing_time, stance_time_initial,stance_time_final);


  // initial footholds all at zero, overwritten anyway
  std::vector<xpp::hyq::Foothold> zero_footholds;
  for (int i=0; i<step_sequence.size(); ++i) {
    xpp::hyq::Foothold f;
    f.leg   = step_sequence.at(i);
    zero_footholds.push_back(f);
  }

  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  // FIXME, maybe initialise support polygon container only with step sequence,
  // not actual steps
  supp_polygon_container.Init(start_stance,
                              zero_footholds, // remove this
                              step_sequence,
                              SupportPolygon::GetDefaultMargins());



  NlpStructure nlp_structure(spline_structure.GetTotalFreeCoeff(),
                             supp_polygon_container.GetNumberOfSteps());

  double robot_height = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/robot_height");
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
                             initial_spline_coeff);


  status = app.OptimizeTNLP(nlp_ipopt_zmp);
  if (status == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;

  }



  final_footholds = nlp_ipopt_zmp->opt_footholds_;
  // FIXME, remove one parameter from this function and apply directly to opt_splines
  spline_structure.AddOptimizedCoefficients(nlp_ipopt_zmp->opt_coeff_, spline_structure.splines_);
  opt_splines = spline_structure.splines_;
}




} /* namespace zmp */
} /* namespace xpp */
