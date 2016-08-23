/*
 * SplineBuilder.h
 *
 *  Created on: May 28, 2014
 *      Author: awinkler
 */
/**
@file    spline_container.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Holds coefficients of multiple fifth-order splines and returns state
         (pos,vel,acc) of appropriate spline for a specific time.
 */
#ifndef _XPP_ZMP_SPLINECONTAINER_H_
#define _XPP_ZMP_SPLINECONTAINER_H_

#include "com_spline.h"
#include <xpp/utils/geometric_structs.h>
#include <vector>

namespace xpp {
namespace zmp {


/** @brief holds multiple splines and knows when these are active in the sequence.
  *
  * Should have no explicit foothold locations or discretization time. Only the framework
  * of splines, where the actual coefficients must be filled through an optimization.
  */
class SplineContainer : public ComSpline {
public:
  SplineContainer();
  virtual ~SplineContainer();

};




} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINECONTAINER_H_
