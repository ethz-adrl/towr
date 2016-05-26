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

class ZmpConstraintBuilder {

public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::hyq::SupportPolygon SupportPolygon;
  typedef xpp::hyq::SupportPolygon::VecSuppLine NodeConstraint;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

public:
  ZmpConstraintBuilder() {};
  ZmpConstraintBuilder(const ContinuousSplineContainer&, double walking_height);
  virtual ~ZmpConstraintBuilder () {};

  /**
   * Initializes the object by pre-calculating the map from optimal coefficients
   * (a,b,c,d) to the zero moment point at every discrete time step t.
   *
   * @param splines the initial map depends only depend on initial state and spline structure.
   * @param walking_height the ZMP is influenced by the height above the ground.
   */
  void Init(const ContinuousSplineContainer&, double walking_height);

  /**
   * Calculates the constraints that keep the ZMP inside the current support
   * polygon.
   *
   * @param s the support polygons from step sequence and location.
   * @return MatrixVectorType m where each set of four rows represents an
   * inequality constraint (m.M*x + m.v > 0) at that discrete time and for that
   * specific support polygon.
   */
  MatVec CalcZmpConstraints(const SupportPolygonContainer& s) const
  {
    CheckIfInitialized();
    return CalcZmpConstraints(x_zmp_map_, y_zmp_map_, s);
  };


private:

  MatVec CalcZmpConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
                            const SupportPolygonContainer&) const;

  static void GenerateNodeConstraint(const NodeConstraint&,
                                     const VecScalar& x_zmp,
                                     const VecScalar& y_zmp,
                                     int row_start,
                                     MatVec& ineq);

  // the zero moment point must always lay on one side of triangle side:
  // p*x_zmp + q*y_zmp + r > stability_margin
  static VecScalar GenerateLineConstraint(const SupportPolygon::SuppLine& l,
                                const VecScalar& x_zmp_M,
                                const VecScalar& y_zmp_M);


  xpp::zmp::ContinuousSplineContainer spline_structure_;
  MatVec x_zmp_map_;
  MatVec y_zmp_map_;

  void CheckIfInitialized() const; // put only in public functions
  bool initialized_ = false;
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_ */
