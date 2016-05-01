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
#include <xpp/zmp/zero_moment_point.h>


namespace xpp {
namespace zmp {

class ZmpConstraint {

public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::hyq::SupportPolygon SupportPolygon;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::Point3d State3d;
  typedef xpp::utils::Point2d State2d;

public:
  ZmpConstraint();
  ZmpConstraint(const ContinuousSplineContainer&, double walking_height);
  virtual ~ZmpConstraint () {};


  MatVec CreateLineConstraints(const SupportPolygonContainer&) const;

  void Init(const ContinuousSplineContainer&, double walking_height);

private:
  xpp::zmp::ContinuousSplineContainer spline_structure_;
  MatVec x_zmp_;
  MatVec y_zmp_;

  MatVec AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                            const SupportPolygonContainer&) const;

  static VecScalar GenerateLineConstraint(const SupportPolygon::SuppLine& l,
                                const VecScalar& x_zmp_M,
                                const VecScalar& y_zmp_M);





  void CheckIfInitialized() const; // put only in public functions
  bool initialized_ = false;
};



} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_ */
