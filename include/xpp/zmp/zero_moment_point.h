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
  typedef xpp::utils::VecScalar VecScalar;

public:
  ZeroMomentPoint () {};
  virtual ~ZeroMomentPoint () {};

  static Vector2d CalcZmp(const State3d& cog, double height);
  static VecScalar CalcZmp(const VecScalar& pos, const VecScalar& acc, double height);

  /**
   * Calculates the position of the ZMP for every discrete time dt along a trajectory.
   *
   * @param cog_spline The CoG spline of 5. order polynomials initialized with specific pos/vel.
   * @param walking_height the height of the CoG above ground.
   * @param dimension what ZMP coordinate you are interested in (X or Y).
   *
   * @return A MatrixVector type m, that together with the spline coefficients x
   * (a,b,c,d) of each spline will return a vector of ZMP positions zmp for each
   * disrete time as: zmp = m.M*x + m.v
   */
  static MatVec ExpressZmpThroughCoefficients(const ContinuousSplineContainer& cog_spline,
                                              double walking_height, int dimension);



  static constexpr double gravity_ = 9.80665; // gravity acceleration [m\s^2]

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZERO_MOMENT_POINT_H_ */
