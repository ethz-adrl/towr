/**
@file    bounds.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Mar 13, 2017
@brief   Brief description
 */

#ifndef OPT_SOLVE_INCLUDE_OPT_BOUNDS_H_
#define OPT_SOLVE_INCLUDE_OPT_BOUNDS_H_


namespace opt {

/** Upper and lower bound on either constraints or optimization variables
  */
struct Bounds {
  Bounds(double lower = 0.0, double upper = 0.0) {
    lower_ = lower;
    upper_ = upper;
  }
  double lower_;
  double upper_;

  void operator+=(double scalar) {
    lower_ += scalar;
    upper_ += scalar;
  }

  void operator-=(double scalar) {
    lower_ -= scalar;
    upper_ -= scalar;
  }
};

static const double inf = 1.0e20;

static const Bounds NoBound          = Bounds(-inf, +inf);
static const Bounds BoundZero        = Bounds( 0.0,  0.0);
static const Bounds BoundGreaterZero = Bounds( 0.0, +inf);
static const Bounds BoundSmallerZero = Bounds(-inf,  0.0);

} // namespace opt

#endif /* OPT_SOLVE_INCLUDE_OPT_BOUNDS_H_ */
