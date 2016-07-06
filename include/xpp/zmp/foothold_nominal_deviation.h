/**
 @file    foothold_nominal_deviation.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

class FootholdNominalDeviation {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef SupportPolygonContainer::VecFoothold VecFoothold;
  typedef utils::StdVecEigen2d StdVecEigen2d;

  FootholdNominalDeviation ();
  virtual ~FootholdNominalDeviation () {}

  StdVecEigen2d GetFeetInBase(const ContinuousSplineContainer&,
                              const SupportPolygonContainer&,
                              StdVecEigen2d& nominal_foothold_b_) const;

//  StdVecEigen2d GetNominalInBase() const;

private:
//  StdVecEigen2d nominal_foothold_b_; ///< positions x-y of nominal position in base frame of that foothold  const double x_nominal_b = 0.3; // 0.4
//  const double x_nominal_b = 0.3; // 0.4
//  const double y_nominal_b = 0.3; // 0.4
//  xpp::hyq::LegDataMap<Eigen::Vector2d> B_r_BaseToNominal;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_FOOTHOLD_NOMINAL_DEVIATION_H_ */
