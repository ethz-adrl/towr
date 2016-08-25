/**
 @file    linear_spline_equations.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Declares LinearSplineEquations class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINEAR_SPLINE_EQUATIONS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINEAR_SPLINE_EQUATIONS_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/zmp/com_spline.h>

namespace xpp {
namespace zmp {

/** Produces linear equations related to CoM spline motion.
  *
  * These equations of the form Mx+b can be used as any type of constraints
  * in optimization problems.
  */
class LinearSplineEquations {
public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::Point2d State2d;
  typedef ComSpline::Ptr ComSplinePtr;

  LinearSplineEquations (const ComSplinePtr& com_spline);
  virtual ~LinearSplineEquations ();

  // refactor write nice comments what these mean
  MatVec MakeInitial(const State2d& init) const;
  MatVec MakeFinal(const State2d& final) const;
  MatVec MakeJunction() const;

  MatVec MakeAcceleration(double weight_x, double weight_y) const;

private:
  ComSplinePtr com_spline_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_LINEAR_SPLINE_EQUATIONS_H_ */
