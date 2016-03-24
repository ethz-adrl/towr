/*
 * supp_triangle_container.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_SUPP_TRIANGLE_CONTAINER_H_

#include <xpp/hyq/supp_triangle.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace hyq {

class SuppTriangleContainer
{
public:
  typedef std::vector<Foothold> Footholds;
  typedef std::vector<SuppTriangle> SuppTriangles;
  typedef std::array<Foothold,3> ArrayF3;
  typedef xpp::utils::MatVec MatVec;

public:
  SuppTriangleContainer ();
  virtual
  ~SuppTriangleContainer ();

public:
  void Init(LegDataMap<Foothold> start_stance,
            const Footholds& footholds,
            const MarginValues& margins);

  bool initialized_ = false;


  Eigen::Vector2d GetCenterOfFinalStance() const;
  MatVec AddLineConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                            const xpp::zmp::ContinuousSplineContainer& zmp_splines) const;


public:
  Footholds footholds_;
  LegDataMap<Foothold> start_stance_;
  MarginValues margins_;

private:
  /**
  @brief Creates the support triangles from footholds and steps.

  @param start_stance Position of feet before walking
  @param steps Position and leg of each new step
  @param stability_margin margin for created support triangles
  @attention modifies start stance
  */
  SuppTriangles GetSupportTriangles(const Footholds& footholds_) const;
  SuppTriangles GetSupportTriangles() const
  {
    return GetSupportTriangles(footholds_);
  }
  void AddLineConstraint(const SuppTriangle::TrLine& l,
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
