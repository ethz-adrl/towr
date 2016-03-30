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
#include <xpp/zmp/qp_optimizer.h>
#include <xpp/zmp/constraints.h>

namespace xpp {
namespace zmp {

class NlpOptimizer {
public:
  NlpOptimizer ();
  virtual
  ~NlpOptimizer ();


  Eigen::VectorXd SolveNlp(Constraints::Footholds& final_footholds,
                           const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
                           const xpp::zmp::QpOptimizer& zmp_optimizer, // TODO, make this more specific
                           const Eigen::VectorXd& initial_spline_coeff = Eigen::Vector2d::Zero());


};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_OPTIMIZER_H_ */
