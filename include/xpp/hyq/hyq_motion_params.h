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
  virtual NominalStance GetNominalStanceInBase() const override;
};

class Walk : public HyqMotionParameters {
public:
  Walk();

  virtual SwingLegCycle GetOneCycle() const;
};

class Trott : public HyqMotionParameters {
public:
  Trott();

  virtual SwingLegCycle GetOneCycle() const;
};

class PushRecovery : public HyqMotionParameters {
public:
  PushRecovery();

  virtual SwingLegCycle GetOneCycle() const;
};

class Camel : public HyqMotionParameters {
public:
  Camel();

  virtual SwingLegCycle GetOneCycle() const;
};

class Bound : public HyqMotionParameters {
public:
  Bound();

  virtual SwingLegCycle GetOneCycle() const;
};


} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_HYQ_HYQ_MOTION_PARAMS_H_ */
