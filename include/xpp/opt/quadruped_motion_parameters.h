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
namespace quad {

class QuadrupedMotionParameters : public MotionParameters {
public:
  QuadrupedMotionParameters();
  static MotionTypePtr MakeMotion(opt::MotionTypeID);

protected:
  // naming convention:, where the circle is is a swingleg, front is right ->.
  // so LF and RH swinging is (bP):  o x
  //                                 x o
  EndeffectorsBool II_;
  EndeffectorsBool PI_;
  EndeffectorsBool bI_;
  EndeffectorsBool IP_;
  EndeffectorsBool Ib_;
  EndeffectorsBool Pb_;
  EndeffectorsBool bP_;
  EndeffectorsBool BI_;
  EndeffectorsBool IB_;
  EndeffectorsBool PP_;
  EndeffectorsBool bb_;
  EndeffectorsBool Bb_;
};

class Walk : public QuadrupedMotionParameters {
public:
  Walk();
};

class Trot : public QuadrupedMotionParameters {
public:
  Trot();
};

class Pace : public QuadrupedMotionParameters {
public:
  Pace();
};

class Bound : public QuadrupedMotionParameters {
public:
  Bound();
};

class PushRecovery : public QuadrupedMotionParameters {
public:
  PushRecovery();
};

} // namespace quad
} // namespace opt
} // namespace xpp

#endif /* XPP_QUADRUPED_MOTION_PARAMETERS_H_ */
