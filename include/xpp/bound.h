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
struct Bound {
  Bound(double lower = 0.0, double upper = 0.0) {
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

static const Bound kNoBound_ = Bound(-1.0e20, +1.0e20);;
static const Bound kEqualityBound_ = Bound(0.0, 0.0);
static const Bound kInequalityBoundPositive_ = Bound(0.0, 1.0e20);

using VecBound = std::vector<Bound>;

}
}

#endif /* XPP_XPP_OPT_INCLUDE_XPP_BOUND_H_ */
