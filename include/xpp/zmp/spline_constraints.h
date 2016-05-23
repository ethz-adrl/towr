/*
 * spline_constraints.h
 *
 *  Created on: Apr 7, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_SPLINE_CONSTRAINTS_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_SPLINE_CONSTRAINTS_H_

#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

class SplineConstraints {
public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::Point2d State;
  static const xpp::utils::Coords3D X = xpp::utils::Coords3D::X;
  static const xpp::utils::Coords3D Y = xpp::utils::Coords3D::Y;
  static const int kDim2d = xpp::utils::kDim2d;

public:
  SplineConstraints (const ContinuousSplineContainer& spline_structure);
  virtual
  ~SplineConstraints () {};

  /**
   * Creates equality constraints of the form Ax=b.
   * Note: The initial position and velocity is defined in ContinousSplineContainer,
   * as those are the e and f coefficients of the initial spline
   */
  MatVec CreateAllSplineConstraints(const Vector2d& intial_acc,
                                    const State& final_state) const;

  MatVec CreateJunctionConstraints() const;
  MatVec InitialAccJerkConstraints(const Vector2d& intial_acc) const;
  MatVec CreateFinalConstraints(const State& final_state) const;

private:
  ContinuousSplineContainer spline_structure_;
  int n_opt_coefficients_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_SPLINE_CONSTRAINT_H_ */
