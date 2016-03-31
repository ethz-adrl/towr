/*
 * supp_triangle_container.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/support_polygon.h>

namespace xpp {
namespace hyq {

class SupportPolygonContainer
{
public:
  typedef std::vector<SupportPolygon> VecSupportPolygon;
  typedef SupportPolygon::VecFoothold VecFoothold;
  typedef xpp::utils::MatVec MatVec;


  struct SplineAndSupport {
    xpp::zmp::ZmpSpline spline_;
    xpp::hyq::SupportPolygon support_;
  };

public:
  SupportPolygonContainer ();
  virtual
  ~SupportPolygonContainer ();

public:
  void Init(LegDataMap<Foothold> start_stance,
            const VecFoothold& footholds,
            const MarginValues& margins);

  bool initialized_ = false;


  Eigen::Vector2d GetCenterOfFinalStance() const;
  MatVec AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                            const xpp::zmp::ContinuousSplineContainer& zmp_splines) const;

  static bool Insert4LSPhase(LegID prev, LegID next);

public:
  VecFoothold footholds_;
  LegDataMap<Foothold> start_stance_;
  MarginValues margins_;

private:
  /**
  @brief Creates the support polygons from footholds and steps.

  @param start_stance Position of feet before walking
  @param steps Position and leg of each new step
  @param stability_margin margin for created support triangles
  @attention modifies start stance
  */
  std::vector<SplineAndSupport> CombineSplineAndSupport(const VecFoothold& footholds, const xpp::zmp::ContinuousSplineContainer& zmp_splines) const;
  std::vector<SplineAndSupport> CombineSplineAndSupport(const xpp::zmp::ContinuousSplineContainer& zmp_splines) const
  {
    return CombineSplineAndSupport(footholds_, zmp_splines);
  }
  void AddLineConstraint(const SupportPolygon::SuppLine& l,
                         const Eigen::RowVectorXd& x_zmp_M,
                         const Eigen::RowVectorXd& y_zmp_M,
                         double x_zmp_v,
                         double y_zmp_v,
                         int& c, MatVec& ineq) const;
  void CheckIfInitialized() const;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_ */
