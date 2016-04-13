/*
 * problem_specification.h
 *
 *  Created on: Apr 13, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PROBLEM_SPECIFICATION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PROBLEM_SPECIFICATION_H_

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

/**
 * Serves as a base class for all modules that give higher level input to the
 * optimizer (cost function, constraints) and bundles all this information.
 * This class does NOT include any NLP/Ipopt specific information.
 */
class ProblemSpecification {

public:
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef SupportPolygonContainer::VecFoothold VecFoothold;

public:
  ProblemSpecification (const SupportPolygonContainer& supp_poly_container,
                        const ContinuousSplineContainer& cog_spline_container);
  virtual
  ~ProblemSpecification ();


  ContinuousSplineContainer zmp_spline_container_;
  SupportPolygonContainer supp_polygon_container_;
  const VecFoothold planned_footholds_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PROBLEM_SPECIFICATION_H_ */
