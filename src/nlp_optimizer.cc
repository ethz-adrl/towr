/*
 * nlp_optimizer.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/nlp_optimizer.h>

#include <xpp/zmp/nlp_ipopt_zmp.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>
#include <xpp/ros/ros_helpers.h>

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
  swing_time_          = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/swing_time");
  stance_time_         = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time");
  stance_time_initial_ = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time_initial");
  stance_time_final_   = xpp::ros::RosHelpers::GetDoubleFromServer("/xpp/stance_time_final");


}


void
NlpOptimizer::SolveNlp(const State& initial_state,
                       const State& final_state,
                       const std::vector<xpp::hyq::LegID>& step_sequence,
                       const VecFoothold& start_stance,
                       VecSpline& opt_splines,
                       VecFoothold& final_footholds,
                       const Eigen::VectorXd& initial_spline_coeff)
{
  // create the general spline structure
  ContinuousSplineContainer spline_structure;
  spline_structure.Init(initial_state.p,
                        initial_state.v ,
                        step_sequence,
                        stance_time_,
                        swing_time_,
                        stance_time_initial_,
                        stance_time_final_);


  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance,
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
                             zmp_publisher_,
                             initial_spline_coeff);

  status_ = app_.OptimizeTNLP(nlp_ipopt_zmp);
  if (status_ == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app_.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app_.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;

  }


  // build the output for the user from the optimized variables
  int n_steps = nlp_ipopt_zmp->opt_footholds_.size();
  final_footholds.resize(n_steps);

  for (int i=0; i<n_steps; ++i) {
    Vector2d f = nlp_ipopt_zmp->opt_footholds_.at(i);
    final_footholds.at(i).p.x() = f.x();
    final_footholds.at(i).p.y() = f.y();
    final_footholds.at(i).leg = step_sequence.at(i);

  }

  spline_structure.AddOptimizedCoefficients(nlp_ipopt_zmp->opt_coeff_);
  opt_splines = spline_structure.splines_;
}




} /* namespace zmp */
} /* namespace xpp */
