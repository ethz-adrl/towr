/**
 @file    com_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the ComMotion class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_MOTION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_MOTION_H_

#include <xpp/utils/geometric_structs.h>

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

  ComMotion ();
  virtual ~ComMotion ();


  virtual int GetTotalFreeCoeff() const = 0;
  virtual VectorXd GetABCDCoeffients() const = 0;


  // this is only in the base spline class
//  virtual Point2d GetCOGxy(double t_global) const;


  /** Creates all the coefficients for the 5th order polyomials from the variables
    */
  virtual void AddOptimizedCoefficients(const VectorXd& optimized_coeff) = 0;


  /** Sets coefficients, so motion ends up at initial position.
    *
    * So at the end of the polynomial (time T), the state of the system will
    * be the start position with zero velocity. Following splines can then just
    * be set with zero coefficient values to stay there as well.
    */
  virtual void SetEndAtStart() = 0;

  // This only works for linear representations!!!.
  /** Produces a vector and scalar, that, multiplied with the spline coefficients
    * a,b,c,d of all splines returns the position of the CoG at time t_local.
    *
    * @param t_local @attention local time of spline. So t_local=0 returns CoG at beginning of this spline.
    * @param id id of current spline
    * @param dim dimension specifying if x or y coordinate of CoG should be calculated
    * @return
    */
  virtual VecScalar ExpressComThroughCoeff(xpp::utils::PosVelAcc, double t_local, int id, Coords3D dim) const = 0;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_MOTION_H_ */
