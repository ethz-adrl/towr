/**
@file    bound.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Mar 13, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_BOUND_H_
#define XPP_XPP_OPT_INCLUDE_XPP_BOUND_H_

#include <vector>

namespace xpp {
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

static const NLPBound kNoBound_        = NLPBound(-1.0e20, +1.0e20);;
static const NLPBound BoundZero        = NLPBound(0.0, 0.0);
static const NLPBound BoundGreaterZero = NLPBound(0.0, +1.0e20);
static const NLPBound BoundSmallerZero = NLPBound(-1.0e20, 0.0);

using VecBound = std::vector<NLPBound>;

}
}

#endif /* XPP_XPP_OPT_INCLUDE_XPP_BOUND_H_ */
