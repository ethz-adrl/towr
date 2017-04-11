/**
 @file    linear_spline_equations.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Declares LinearSplineEquations class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUATIONS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUATIONS_H_

#include <array>
#include <cstddef>
#include <memory>
#include <vector>
#include <sys/types.h>
#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/matrix_vector.h>

namespace xpp {
namespace opt {

class BaseMotion;
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
  using MotionDerivatives = std::vector<MotionDerivative>;
  using ComSplinePtr      = std::shared_ptr<ComSpline>;
  using BaseMotionPtr     = std::shared_ptr<BaseMotion>;
  using ValXY             = std::array<double,2>;

  /** @attention ComMotion is downcast to ComSpline.
    */
  LinearSplineEquations();
  LinearSplineEquations (const ComSplinePtr&);
  virtual ~LinearSplineEquations ();


  /** M*x + v gives the difference to the desired initial state
    *
    * @param init desired initial position, velocity and acceleration.
    */
  MatVec MakeInitial(const StateLin2d& init) const;

  /** M*x + v gives the difference to the desired final state
    *
    * @param final desired final position, velocity and acceleration.
    */
  MatVec MakeFinal(const StateLin2d& final, const MotionDerivatives& ) const;

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
    * @param weight_xy larger value produces larger cost for x motion.
    */
  Eigen::MatrixXd MakeAcceleration(const ValXY& weight_xy) const;
  Eigen::MatrixXd MakeJerk(const ValXY& weight_xy) const;

private:
  ComSplinePtr com_spline_;

  template<std::size_t N>
  std::array<double,N> CalcExponents(double t) const;
};


template<std::size_t N>
std::array<double,N>
LinearSplineEquations::CalcExponents(double t) const
{
  std::array<double,N> exp = {{ 1.0, t }};
  for (uint e = 2; e < N; ++e)
    exp[e] = exp[e-1] * t;
  return exp;
}


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_LINEAR_SPLINE_EQUATIONS_H_ */
