/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the ComMotion class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_MOTION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_MOTION_H_

#include <xpp/utils/geometric_structs.h>
#include <memory>

namespace xpp {
namespace zmp {

/** Abstracts the Center of Mass (CoM) motion of any system.
  *
  * This class is responsible for providing a common interface to represent
  * the motion of a system. Specific implementation can for example use
  * splines or solutions of the Equation of Motion as representation.
  */
class ComMotion {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::VecScalar VecScalar;
  typedef xpp::utils::Coords3D Coords3D;
  typedef xpp::utils::PosVelAcc PosVelAcc;
  typedef xpp::utils::Point2d Point2d;
  typedef std::shared_ptr<ComMotion> Ptr;

  ComMotion ();
  virtual ~ComMotion ();


  virtual int GetTotalFreeCoeff() const = 0;
  virtual VectorXd GetABCDCoeffients() const = 0;

  virtual Point2d GetCOGxy(double t_global) const = 0;


  /** Creates all the coefficients for the 5th order polynomials from the variables
    */
  virtual void AddOptimizedCoefficients(const VectorXd& optimized_coeff) = 0;

  virtual double GetTotalTime() const = 0;

  /** If the trajectory has to be discretized, use this for consistent time steps.
   *  t(0)------t(1)------t(2)------...------t(N-1)---|------t(N)
   *
   *  so first and last time are t0 and and tN, but there might be a
   *  timestep > delta t before the last node.
   */
  std::vector<double> GetDiscretizedGlobalTimes() const;
  int GetTotalNodes() const;


  /** Sets coefficients, so motion ends up at initial position.
    *
    * So at the end of the polynomial (time T), the state of the system will
    * be the start position with zero velocity. Following splines can then just
    * be set with zero coefficient values to stay there as well.
    */
  virtual void SetEndAtStart() = 0;

private:
  static constexpr double eps_ = 1e-10; // maximum inaccuracy when adding double numbers
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_MOTION_H_ */
