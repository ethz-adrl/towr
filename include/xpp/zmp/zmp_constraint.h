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
  typedef xpp::hyq::SupportPolygon SupportPolygon;
  typedef xpp::hyq::Foothold Foothold;

public:
  ZmpConstraint() {}; // FIXME remove this
  ZmpConstraint (xpp::zmp::ContinuousSplineContainer spline_container,
                 xpp::hyq::SupportPolygonContainer supp_polygon_container);
  virtual
  ~ZmpConstraint ();

  MatVec AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp) const;

  xpp::hyq::SupportPolygonContainer supp_polygon_container_;

  std::vector<Foothold> GetFootholds() const { return supp_polygon_container_.GetFootholds(); };
  Foothold GetStartStance(hyq::LegID leg) const { return supp_polygon_container_.start_stance_[leg]; };

private:
  xpp::zmp::ContinuousSplineContainer spline_container_;
  std::vector<hyq::SupportPolygon> CreateSupportPolygonsWith4LS() const;
  static void AddLineConstraint(const SupportPolygon::SuppLine& l,
                                const Eigen::RowVectorXd& x_zmp_M,
                                const Eigen::RowVectorXd& y_zmp_M,
                                double x_zmp_v,
                                double y_zmp_v,
                                int& c, MatVec& ineq);
};



} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_ */
