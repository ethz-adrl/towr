/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the ZeroMomentPoint class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZERO_MOMENT_POINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZERO_MOMENT_POINT_H_

#include <xpp/utils/geometric_structs.h>
#include <Eigen/Sparse>

namespace xpp {
namespace zmp {

class ComMotion;

/** Calculates the Zero Moment Point for a specific motion defined by coefficients.
  *
  * The ZMP is defined as:
  * p = x - height/(gravity_+zdd)*xdd
  */
class ZeroMomentPoint {
public:
  typedef xpp::utils::MatVec MatVec;
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::Point3d State3d;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::utils::Coords3D Coords;
  typedef Eigen::SparseVector<double, Eigen::RowMajor> JacobianRow;
  typedef Eigen::SparseMatrix<double, Eigen::RowMajor> Jacobian;

public:
  ZeroMomentPoint () {};
  virtual ~ZeroMomentPoint () {};

  static Vector2d  CalcZmp(const State3d& cog, double height);
  template <typename T>
  static T CalcZmp(const T& pos, const T& acc, double height);

  /** Creates a linear approximation of the ZMP w.r.t the current motion coefficients uc
    *
    * The output of this the Jacobian J(t, uc) w.r.t the current motion
    * coefficients u_m and the offset x0(t), which is the ZMP evaluated
    * for zero motion coefficients at each time t.
    *
    * The position of the ZMP can be reconstructed with this information as:
    * p = J*(u-uc) + x0
    *
    * This yields a vector of ZMP positions for every discretized time t.

    * @param s a CoM motion described by current motion coefficients uc.
    * @param walking_height the height of the CoM above ground.
    * @param dimension what ZMP coordinate you are interested in (X or Y).
    */
  static MatVec GetLinearApproxWrtMotionCoeff(const ComMotion& x, double walking_height, Coords dimension);

  static Jacobian GetJacobianWrtCoeff(const ComMotion& x, double walking_height, Coords dimension);

private:
  static constexpr double kGravity = 9.80665; // gravity acceleration [m\s^2]
};


template <typename T> T
ZeroMomentPoint::CalcZmp(const T& pos, const T& acc, double height)
{
  const double z_acc = 0.0; // TODO: calculate z_acc based on foothold height
  double k = height/(kGravity+z_acc);
  return pos - k*acc;
}

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZERO_MOMENT_POINT_H_ */
