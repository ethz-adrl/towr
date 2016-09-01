/**
 @file    zmp_contraint_builder.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Declares the ZmpConstraintBuilder class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

namespace xpp {
namespace hyq {
class SupportPolygonContainer;
}
}

namespace xpp {
namespace zmp {

class ComMotion;

/** @brief Calculates the value and jacobian of the stability constraint.
  *
  * This class is responsible for providing the distance of the ZMP to the
  * boarders of the support polygon for each discrete time t. Additionally
  * it supplies the jacobian of this distance with respect to the motion
  * coefficients and with respect to the contact positions.
  */
class ZmpConstraintBuilder {
public:
  using SupportPolygonContainer = xpp::hyq::SupportPolygonContainer;
  using SuppPolygonPtrU = std::unique_ptr<xpp::hyq::SupportPolygonContainer>;
  using MotionPtrU      = std::unique_ptr<ComMotion>;
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  ZmpConstraintBuilder();
  /** @param motion  The CoM motion for which stability should be ensured.
    * @param support The support polygons comprised of the footsteps.
    * @param height  The height at which the robot is walking.
    * @param dt      The timestep[s] at which the constraint is enforced.
    */
  ZmpConstraintBuilder(const ComMotion& motion, const SupportPolygonContainer& support,
                       double height, double dt);
  virtual ~ZmpConstraintBuilder ();

  void Init(const ComMotion&, const SupportPolygonContainer&,
            double walking_height, double dt);

  void Update(const VectorXd& motion_coeff, const VectorXd& footholds);

  Jacobian GetJacobianWrtMotion() const;
  Jacobian GetJacobianWrtContacts() const;
  VectorXd GetDistanceToLineMargin() const;

private:
  MotionPtrU com_motion_;
  SuppPolygonPtrU supp_polygon_;

  /** The Jacobian for the current motion and contact coefficients for
    * these current parameters  for each discrete time t along the trajectory
    * and for every line at this discrete time t.
    */
  void CalcJacobians();
  int GetNumberOfConstraints() const;

  int n_constraints_ = 0;

  double walking_height_;
  Jacobian jac_zmpx_0_; ///< Jacobian of ZMP in x direction evaluated at spline coefficient values of zero
  Jacobian jac_zmpy_0_; ///< Jacobian of ZMP in y direction evaluated at spline coefficient values of zero

  Jacobian jac_wrt_motion_;
  Jacobian jac_wrt_contacts_;

  /** @returns the times at which two phases have disjoint support polygons
    */
  std::vector<double> GetTimesDisjointSwitches() const;

  /** @returns times at which the ZMP constraint is evaluated.
    * @param dt       The time between two evaluation nodes of the trajectory.
    * @param t_switch The times where the motion switches between disjoint suppport areas
    *
    * This is necessary, as we are forcing the CoM to be inside a support polygon
    * at all times. However, for disjoint support polyons (e.g. swinging the
    * LF leg -> RF leg) there is a gap between these as we are shrinking them
    * by a stability margin. We relax the stability constraint at theses nodes
    * for a short time @p t_swithc to allow the motion to smoothly traverse
    * to the next polygon.
    */
  std::vector<double> GetTimesForConstraitEvaluation(double dt, double t_switch) const;
  std::vector<double> times_; ///< times at which constraint should be evaluated

  friend class ZmpConstraintBuilderTest_GetTimesDisjointSwitches_Test;
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_SRC_ZMP_CONSTRAINT_H_ */
