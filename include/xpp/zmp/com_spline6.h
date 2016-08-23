/**
@file    com_spline6.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Declares ComSpline6, which realizes a ComSpline
 */

#ifndef _XPP_ZMP_COMSPLINE6_H_
#define _XPP_ZMP_COMSPLINE6_H_

#include "com_spline.h"

namespace xpp {
namespace zmp {

/** Represents the center of mass motion with 6 coefficients per polynomial
  *
  * This class represents a collection of fifth order polynomials
  * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f with no constraint at the junction
  * between the polynomials.
  */
class ComSpline6 : public ComSpline {
public:
  ComSpline6();
  virtual ~ComSpline6();

};




} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_COMSPLINE6_H_
