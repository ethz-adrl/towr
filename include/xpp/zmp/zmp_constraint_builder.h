/*
 * zmp_constraint.h
 *
 *  Created on: Apr 4, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_

#include "zero_moment_point.h"
#include "com_spline.h"
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

class ZmpConstraintBuilder {

public:
  typedef ComSpline::Ptr ComSplinePtr;
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::utils::VecScalarScalar VecScalarScalar;
  typedef xpp::utils::MatVecVec MatVecVec;
  typedef xpp::hyq::SupportPolygon SupportPolygon;
  typedef xpp::hyq::SupportPolygon::VecSuppLine NodeConstraint;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef xpp::hyq::LegID LegID;
  typedef Eigen::Vector2d Vector2d;
  typedef Eigen::VectorXd VectorXd;
  using SuppLine = xpp::hyq::SupportPolygon::SuppLine;

  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using JacobianRow = ComMotion::JacobianRow;

public:
  ZmpConstraintBuilder() {};
  ZmpConstraintBuilder(const ComSplinePtr&, const SupportPolygonContainer&, const double walking_height);
  virtual ~ZmpConstraintBuilder () {};

  /** Initializes the object by pre-calculating the map from optimal coefficients
    * (a,b,c,d) to the zero moment point at every discrete time step t.
    *
    * @param splines the initial map depends only depend on initial state and spline structure.
    * @param walking_height the ZMP is influenced by the height above the ground.
    */
  void Init(const ComSplinePtr&, const SupportPolygonContainer& supp, double walking_height);

  void Update(const VectorXd& motion_coeff, const VectorXd& footholds);

  /** Calculates the constraints that keep the ZMP inside the current support
    * polygon.
    *
    * @param s the support polygons from step sequence and location.
    * @return MatrixVectorType m where each set of four rows represents an
    * inequality constraint (m.M*x + m.v > 0) at that discrete time and for that
    * specific support polygon. This constraint is evaluated by multiplying with
    * the spline coefficients x.
    */
  // refactor returns jacobian, rename
//  MatVecVec CalcZmpConstraints(const SupportPolygonContainer& s) const;


  /////////////////////new stuff///////////////////////////////

  /** The Jacobian for the current motion and contact coefficients for
    * these current parameters  for each discrete time t along the trajectory
    * and for every line at this discrete time t.
    */
  void CalcJacobians();

  Jacobian GetJacobianWrtMotion() const;
  Jacobian GetJacobianWrtContacts() const;


  VectorXd GetDistanceToLineMargin() const;

  // refactor make private again
  std::unique_ptr<ComSpline> spline_structure_;
  SupportPolygonContainer supp_polygon_;

private:
  double GetDistanceToLineMargin(const Vector2d& zmp, SuppLine line) const;

//  MatVecVec CalcZmpConstraints(const MatVec& x_zmp, const MatVec& y_zmp,
//                            const SupportPolygonContainer&) const;
//
//  static void GenerateNodeConstraint(const NodeConstraint&,
//                                     const VecScalar& x_zmp,
//                                     const VecScalar& y_zmp,
//                                     int row_start,
//                                     MatVecVec& ineq);
//
//  // the zero moment point must always lay on one side of triangle side:
//  // p*x_zmp + q*y_zmp + r > stability_margin
//  static VecScalarScalar GenerateLineConstraint(const SupportPolygon::SuppLine& l,
//                                                const VecScalar& x_zmp_M,
//                                                const VecScalar& y_zmp_M);
//
//  /** Check if this spline needs a four leg support phase to go to next spline.
//    *
//    * Reducing the support polygons by a margin creates disjoint support triangles
//    * when switching to diagonally opposite swing legs. This causes the optimizer
//    * to fail, if the constraints are not disregarded at theses times.
//    *
//    * @param t current time of trajectory
//    * @param curr_spline active spline at the moment
//    * @param support polygon
//    * @return true if there are no constrains on current spline at time t
//    */
//  bool DisjSuppSwitch(double t, const ComPolynomial& curr_spline, const SupportPolygonContainer&) const;
//
//  bool DisjointSuppPolygonsAtBeginning(int step, const SupportPolygonContainer&) const;
//  bool DisjointSuppPolygonsAtEnd(int step, const SupportPolygonContainer&) const;
//
//
//
//  static bool Insert4LSPhase(LegID prev, LegID next);



  double walking_height_;
  Jacobian jac_zmpx_0_; ///< Jacobian of ZMP in x direction evaluated at spline coefficient values of zero
  Jacobian jac_zmpy_0_; ///< Jacobian of ZMP in y direction evaluated at spline coefficient values of zero

  Jacobian jac_wrt_motion_;
  Jacobian jac_wrt_contacts_;

  void CheckIfInitialized() const; // put only in public functions
  bool initialized_ = false;
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_ */
