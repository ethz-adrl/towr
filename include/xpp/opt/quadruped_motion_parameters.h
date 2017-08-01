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

class SingleMotionParameters : public MotionParameters {
public:
  SingleMotionParameters() {



    ee_splines_per_swing_phase_ = 2; // spring_clean_ this breaks duration derivatives
    force_splines_per_stance_phase_ = 6;

    robot_ee_ = { EEID::E0 };
    dt_range_of_motion_ = 0.15;

    // dynamic model for HyQ
    mass_    = 80;
    interia_ = buildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);
    force_limit_ = 10000.0; // [N]

    // range of motion specifictions for HyQ
    const double z_nominal_b = -0.58;
    nominal_stance_.SetCount(robot_ee_.size());
    nominal_stance_.At(EEID::E0) = PosXYZ( 0.0, 0.0, z_nominal_b);
    max_dev_xy_ << 0.15, 0.15, 0.1;

    // spring_clean_ just the addition is used anyway
    double t_swing  = 0.1;
    double t_stance = 0.5;
    contact_timings_ = {t_stance, t_swing, t_stance, t_swing};
    constraints_ = {
        State,
        JunctionCom,
        RomBox,
//        Dynamic,
    };



    dt_base_polynomial_    = std::accumulate(contact_timings_.begin(),
                                             contact_timings_.end(),
                                             0.0); //s 0.05

    // zmp_ since derivative of acceleration is nonsmooth at junctions, pay attention
    // to never evaluate at junction of base polynomial directly
    // (what i'm doing now! :-(
    // must make sure every polynomial is at least evaluated once
    dt_dynamic_constraint_ = dt_base_polynomial_/2;
  }


};

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
  // 2 swinglegs
  EndeffectorsBool Pb_;
  EndeffectorsBool bP_;
  EndeffectorsBool BI_;
  EndeffectorsBool IB_;
  EndeffectorsBool PP_;
  EndeffectorsBool bb_;
  // 3 swinglegs
  EndeffectorsBool Bb_;
  EndeffectorsBool BP_;
  EndeffectorsBool bB_;
  EndeffectorsBool PB_;
  // flight-phase
  EndeffectorsBool BB_;


  void SetContactSequence(double p_overlap_lateral,
                          double p_overlap_opposite);
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
