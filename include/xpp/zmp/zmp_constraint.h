/*
 * zmp_constraint.h
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

class ZmpConstraint {

public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::hyq::SupportPolygon SupportPolygon;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

public:
  ZmpConstraint (ContinuousSplineContainer spline_container, double walking_height);
  virtual
  ~ZmpConstraint () {};


  MatVec CreateLineConstraints(const SupportPolygonContainer& supp_polygon_container) const;

private:
  xpp::zmp::ContinuousSplineContainer spline_container_;
  MatVec x_zmp_;
  MatVec y_zmp_;
  MatVec AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                            const SupportPolygonContainer& supp_polygon_container) const;
  std::vector<SupportPolygon> CreateSupportPolygonsWith4LS(
      const SupportPolygonContainer& supp_polygon_container) const;

  static VecScalar GenerateLineConstraint(const SupportPolygon::SuppLine& l,
                                const VecScalar& x_zmp_M,
                                const VecScalar& y_zmp_M);

  MatVec ExpressZmpThroughCoefficients(double walking_height, int dim) const;
};



} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_ */
