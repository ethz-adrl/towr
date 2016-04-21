/*
 * nlp_optimizer.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_


#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/spline_constraints.h>

namespace xpp {
namespace zmp {

class NlpOptimizer {
public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::zmp::SplineConstraints::State State;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

public:
  NlpOptimizer () {};
  virtual
  ~NlpOptimizer () {};


  Eigen::VectorXd SolveNlp(const Vector2d& initial_acc,
                           const State& final_state,
                           StdVecEigen2d& final_footholds,
                           const ContinuousSplineContainer& spline_structure,
                           const SupportPolygonContainer& supp_polygon_container,
                           double walking_height,
                           const Eigen::VectorXd& initial_spline_coeff = Eigen::Vector2d::Zero());


};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_ */
