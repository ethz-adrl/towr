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

NlpOptimizer::NlpOptimizer ()
{
  // TODO Auto-generated constructor stub

}

NlpOptimizer::~NlpOptimizer ()
{
  // TODO Auto-generated destructor stub
}



Eigen::VectorXd
NlpOptimizer::SolveNlp(Eigen::VectorXd& final_footholds,
                       const xpp::hyq::SuppTriangleContainer& supp_triangle_container,
                       const xpp::zmp::ContinuousSplineContainer splines_container,
                       const xpp::zmp::QpOptimizer& zmp_optimizer, // TODO, make this more specific
                       const Eigen::VectorXd& initialization_values)
{
  Ipopt::IpoptApplication app;
  Ipopt::ApplicationReturnStatus status = app.Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }


  Ipopt::SmartPtr<Ipopt::NlpIpoptZmp> nlp_ipopt_zmp = new Ipopt::NlpIpoptZmp();
  nlp_ipopt_zmp->SetupNlp(zmp_optimizer.cf_,
                          zmp_optimizer.eq_,
                          zmp_optimizer.ineq_ipopt_,
                          zmp_optimizer.ineq_ipopt_vx_,
                          zmp_optimizer.ineq_ipopt_vy_,
                          splines_container, // use splines directly
                          supp_triangle_container,
                          zmp_optimizer, // remove this
                          initialization_values);


  // FIXME make sure the zmp_optimizer member variables is already properly filled!!!
  status = app.OptimizeTNLP(nlp_ipopt_zmp);
  if (status == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;

  }

  final_footholds = nlp_ipopt_zmp->x_final_footholds_;
  return nlp_ipopt_zmp->x_final_spline_coeff_;
}















} /* namespace zmp */
} /* namespace xpp */
