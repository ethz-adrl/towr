/**
 @file    linear_spline_equations.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Declares LinearSplineEquations class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINEAR_SPLINE_EQUATIONS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINEAR_SPLINE_EQUATIONS_H_

#include <xpp/utils/geometric_structs.h>
#include <memory>

namespace xpp {
namespace zmp {

class ComMotion;
class ComSpline; // at some point get rid of this

/** Produces linear equations related to CoM spline motion coefficients x.
  *
  * All the provided functions return a matrix M and a vector v of dimensions
  * M=(m or n) x n and v=(m or n) x 1
  * where:
  * m = e.g. number of constraints
  * n = e.g. number of spline coefficients
  */
class LinearSplineEquations {
public:
  using MatVec            = xpp::utils::MatVec;
  using State2d           = xpp::utils::Point2d;
  using MotionDerivatives = std::vector<xpp::utils::MotionDerivative>;
  using ComSplinePtrU     = std::unique_ptr<ComSpline>;

  /** @attention ComMotion is downcast to ComSpline.
    */
  LinearSplineEquations (const ComMotion& com_spline);
  virtual ~LinearSplineEquations ();


  /** M*x + v gives the difference to the desired initial state
    *
    * @param init desired initial position, velocity and acceleration.
    */
  MatVec MakeInitial(const State2d& init) const;

  /** M*x + v gives the difference to the desired final state
    *
    * @param final desired final position, velocity and acceleration.
    */
  MatVec MakeFinal(const State2d& final, const MotionDerivatives& ) const;

  /** M*x + v gives the difference at the polynomial junctions of the spline
    *
    * A spline made up of 3 polynomials has 2 junctions, and for each of these
    * the position, velocity and acceleration difference in x-y is returned,
    * resulting in m = (number of splines-1) * 3 * 2
    */
  MatVec MakeJunction() const;

  /** xT*M*x + xT*v gives the scalar total acceleration cost with these x.
    *
    * To turn the motion derivatives into costs they are weighed according to the
    * directions (x,y). Usually lateral motions are penalized more (bigger weight)
    * than forward backwards motions.
    *
    * @param weight_x larger value produces larger cost for x motion.
    * @param weight_y larger value produces larger cost for y motion.
    */
  Eigen::MatrixXd MakeAcceleration(double weight_x, double weight_y) const;
  Eigen::MatrixXd MakeJerk(double weight_x, double weight_y) const;

private:
  ComSplinePtrU com_spline_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINEAR_SPLINE_EQUATIONS_H_ */
