/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the ZeroMomentPoint class
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_ZERO_MOMENT_POINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_ZERO_MOMENT_POINT_H_

#include <xpp/utils/base_state.h>
#include <Eigen/Sparse>
#include <memory>

namespace xpp {
namespace opt {

class ComMotion;

/** Calculates the Zero Moment Point for a specific motion defined by coefficients.
  *
  * The ZMP is defined as:
  * p = x - height/(gravity_+zdd)*xdd
  */
class ZeroMomentPoint {
public:
  typedef Eigen::Vector2d Vector2d;
  typedef xpp::utils::BaseLin3d State3d;
  typedef xpp::utils::Coords3D Coords;
  typedef Eigen::SparseVector<double, Eigen::RowMajor> JacobianRow;
  typedef Eigen::SparseMatrix<double, Eigen::RowMajor> Jacobian;
  typedef std::unique_ptr<ComMotion> ComMotionPtr;

public:
  ZeroMomentPoint ();
  ZeroMomentPoint (const ComMotion& x, const std::vector<double>& times, double height);
  virtual ~ZeroMomentPoint ();

  /** @brief Provide the required info to calculate the ZMP.
    * @param x       The CoM motion described by current motion coefficients u_m.
    * @param times   All time instances at which the ZMP Jacobian is calculated.
    * @param height  The position of the of the CoM above ground.
    */
  void Init(const ComMotion& x, const std::vector<double>& times, double height);

  /** @brief Calculate the Jacobian of the ZMP position.
    * @param dim   The coordinate (X or Y) of the ZMP.
    * @return      The Jacobian of the ZMP position w.r.t to motion coefficients.
    *
    * The output is the Jacobian J(t, uc) w.r.t the current motion coefficients.
    * The Jacobian has @p times.size() rows and @p u_m.size() cols.
    */
  Jacobian GetJacobianWrtCoeff(Coords dimension) const;

  static Vector2d  CalcZmp(const State3d& cog, double height);
  template <typename T>
  static T CalcZmp(const T& pos, const T& acc, double height);

private:
  ComMotionPtr com_motion_;
  std::vector<double> times_;
  double height_;
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

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_ZERO_MOMENT_POINT_H_ */
