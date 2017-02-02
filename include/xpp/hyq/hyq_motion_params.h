/**
 @file    hyq_motion_params.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_HYQ_HYQ_MOTION_PARAMS_H_
#define XPP_XPP_OPT_INCLUDE_HYQ_HYQ_MOTION_PARAMS_H_

#include <xpp/opt/motion_parameters.h>

namespace xpp {
namespace hyq {

class HyqMotionParameters : public opt::MotionParameters {
public:
  HyqMotionParameters();
};

class Walk : public HyqMotionParameters {
public:
  Walk();
};

class Trott : public HyqMotionParameters {
public:
  Trott();
};

class PushRecovery : public HyqMotionParameters {
public:
  PushRecovery();
};

class Camel : public HyqMotionParameters {
public:
  Camel();
};

class Bound : public HyqMotionParameters {
public:
  Bound();
};


} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_HYQ_HYQ_MOTION_PARAMS_H_ */
