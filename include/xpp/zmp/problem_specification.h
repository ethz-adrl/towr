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
  typedef xpp::hyq::Foothold Foothold;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::hyq::LegID LegID;

public:
  explicit ProblemSpecification (const SupportPolygonContainer& supp_poly_container,
                                 const ContinuousSplineContainer& cog_spline_container);
  virtual ~ProblemSpecification ();

  ContinuousSplineContainer GetSplineContainer() const { return zmp_spline_container_; };
  Foothold GetPlannedFoothold(size_t i) const { return planned_footholds_.at(i); };
  VecFoothold GetPlannedFootholds() const { return planned_footholds_; };
  Foothold GetStartStance(LegID leg) const { return supp_polygon_container_.GetStartStance()[leg]; };


protected:
  SupportPolygonContainer supp_polygon_container_;
  ContinuousSplineContainer zmp_spline_container_;
  const VecFoothold planned_footholds_;

protected:
  Eigen::VectorXd FixFootholdPosition(const StdVecEigen2d& footholds) const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PROBLEM_SPECIFICATION_H_ */
