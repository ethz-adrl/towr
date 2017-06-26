/**
 @file    linear_spline_equations.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Declares LinearSplineEquations class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUATIONS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUATIONS_H_

#include <array>
#include <Eigen/Dense>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/matrix_vector.h>

#include "polynomial_spline.h"

namespace xpp {
namespace opt {

class PolynomialSpline;

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
  using MotionDerivatives = std::vector<MotionDerivative>;
  using ValXYZ            = std::array<double,3>;

  LinearSplineEquations();
  LinearSplineEquations (const PolynomialSpline&);
  virtual ~LinearSplineEquations ();

  /** M*x + v gives the difference to the state
    *
    * @param state desired position, velocity and acceleration.
    */
  MatVec MakeStateConstraint(const StateLin3d& state, double t, const MotionDerivatives& ) const;

  /** M*x + v gives the difference at the polynomial junctions of the spline
    *
    * A spline made up of 3 polynomials has 2 junctions, and for each of these
    * the position and velocity difference in x-y is returned,
    * resulting in m = (number of splines-1) * 3 * 2
    */
  MatVec MakeJunction() const;

  /** xT*M*x + xT*v gives the scalar total acceleration cost with these x.
    *
    * To turn the motion derivatives into costs they are weighed according to the
    * directions (x,y). Usually lateral motions are penalized more (bigger weight)
    * than forward backwards motions.
    *
    * @param weight which acceleration to avoid (x,y,z)
    */
  Eigen::MatrixXd MakeCostMatrix(const ValXYZ& weights, MotionDerivative) const;

private:
  PolynomialSpline poly_spline_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUATIONS_H_ */
