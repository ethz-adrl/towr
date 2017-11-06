/**
@file    bound.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Mar 13, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_BOUND_H_
#define XPP_XPP_OPT_INCLUDE_XPP_BOUND_H_


namespace opt {

/** Upper and lower bound on either constraints or optimization variables
  */
struct NLPBound {
  NLPBound(double lower = 0.0, double upper = 0.0) {
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

static const NLPBound NoBound          = NLPBound(-inf, +inf);
static const NLPBound BoundZero        = NLPBound( 0.0,  0.0);
static const NLPBound BoundGreaterZero = NLPBound( 0.0, +inf);
static const NLPBound BoundSmallerZero = NLPBound(-inf,  0.0);

} // namespace opt

#endif /* XPP_XPP_OPT_INCLUDE_XPP_BOUND_H_ */
