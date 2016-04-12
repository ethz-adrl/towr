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

namespace xpp {
namespace zmp {


Eigen::VectorXd
NlpOptimizer::SolveNlp(StdVecEigen2d& final_footholds,
                       const ContinuousSplineContainer& spline_structure,
                       const SupportPolygonContainer& supp_triangle_container,
                       double walking_height,
                       const Eigen::VectorXd& initial_spline_coeff)
{
  Ipopt::IpoptApplication app;
  app.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  Ipopt::ApplicationReturnStatus status = app.Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }


  Constraints constraints(supp_triangle_container, spline_structure, walking_height);
  CostFunction cost_function(spline_structure);
  Objective objective(cost_function);

  Ipopt::SmartPtr<Ipopt::NlpIpoptZmp> nlp_ipopt_zmp =
      new Ipopt::NlpIpoptZmp(objective,
                             constraints,
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
  return nlp_ipopt_zmp->opt_coeff_;
}




} /* namespace zmp */
} /* namespace xpp */
