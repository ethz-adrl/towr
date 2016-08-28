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

using namespace xpp::utils::coords_wrapper; //kPos,kVel,kAcc,kJerk

namespace xpp {
namespace zmp {

class ComSpline;

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
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::Point2d State2d;
  typedef std::unique_ptr<ComSpline> ComSplinePtr;
  typedef std::shared_ptr<ComSpline> ComSplinePtrShared;
  typedef std::vector<MotionDerivative> Derivatives;

  LinearSplineEquations (const ComSplinePtrShared& com_spline);
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
  MatVec MakeFinal(const State2d& final) const;

  /** M*x + v gives the difference at the polynomial junctions of the spline
    *
    * A spline made up of 3 polynomials has 2 junctions, and for each of these
    * the position, velocity and acceleration difference in x-y is returned,
    * resulting in m = (number of splines-1) * 3 * 2
    */
  MatVec MakeJunction() const;

  /** xT*M*x + xT*v gives the scalar total acceleration cost with these x.
    *
    * To turn the acceleration into costs they are weighed according to the
    * directions (x,y). Usually lateral motions are penalized more (bigger weight)
    * than forward backwards motions.
    *
    * @param weight_x larger value produces larger cost for x motion.
    * @param weight_y larger value produces larger cost for y motion.
    */
  MatVec MakeAcceleration(double weight_x, double weight_y) const;

private:
  ComSplinePtr com_spline_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINEAR_SPLINE_EQUATIONS_H_ */
