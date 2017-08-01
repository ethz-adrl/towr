/**
 @file    quadruped_motion_parameterss.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Brief description
 */

#ifndef XPP_QUADRUPED_MOTION_PARAMETERS_H_
#define XPP_QUADRUPED_MOTION_PARAMETERS_H_

#include <xpp/endeffectors.h>
#include "motion_parameters.h"

namespace xpp {
namespace opt {

class MonopedMotionParameters : public MotionParameters {
public:
  MonopedMotionParameters();
  virtual ~MonopedMotionParameters() {};
};

class QuadrupedMotionParameters : public MotionParameters {
public:
  QuadrupedMotionParameters();

protected:
//  // naming convention:, where the circle is is a swingleg, front is right ->.
//  // so LF and RH swinging is (bP):  o x
//  //                                 x o
//  EndeffectorsBool II_;
//  EndeffectorsBool PI_;
//  EndeffectorsBool bI_;
//  EndeffectorsBool IP_;
//  EndeffectorsBool Ib_;
//  // 2 swinglegs
//  EndeffectorsBool Pb_;
//  EndeffectorsBool bP_;
//  EndeffectorsBool BI_;
//  EndeffectorsBool IB_;
//  EndeffectorsBool PP_;
//  EndeffectorsBool bb_;
//  // 3 swinglegs
//  EndeffectorsBool Bb_;
//  EndeffectorsBool BP_;
//  EndeffectorsBool bB_;
//  EndeffectorsBool PB_;
//  // flight-phase
//  EndeffectorsBool BB_;


//  void SetContactSequence(double p_overlap_lateral,
//                          double p_overlap_opposite);
};

} // namespace opt
} // namespace xpp

#endif /* XPP_QUADRUPED_MOTION_PARAMETERS_H_ */
