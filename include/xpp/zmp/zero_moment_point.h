/*
 * zero_moment_point.h
 *
 *  Created on: Apr 30, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZERO_MOMENT_POINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZERO_MOMENT_POINT_H_

#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

class ZeroMomentPoint {
public:
  typedef xpp::utils::MatVec MatVec;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::Point3d State3d;

public:
  ZeroMomentPoint ();
  virtual ~ZeroMomentPoint ();

  static Vector2d CalcZmp(const State3d& cog, double height);
  static MatVec ExpressZmpThroughCoefficients(const ContinuousSplineContainer&,
                                              double walking_height, int dim);

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZERO_MOMENT_POINT_H_ */
