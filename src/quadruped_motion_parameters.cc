/**
 @file    hyq_motion_params.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Brief description
 */

#include <xpp/opt/quadruped_motion_parameters.h>

#include <initializer_list>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

#include <xpp/endeffectors4.h>

namespace xpp {
namespace opt {
namespace quad{

QuadrupedMotionParameters::QuadrupedMotionParameters ()
{
  geom_walking_height_ = 0.58;
//  offset_geom_to_com_ << -0.02230, -0.00010, 0.03870;
//  offset_geom_to_com_ << -0.03, 0.02, 0.0;
  offset_geom_to_com_ << 0,0,0;
  robot_ee_ = { EEID::E0, EEID::E1, EEID::E2, EEID::E3 };

  const double x_nominal_b = 0.34;
  const double y_nominal_b = 0.34;
  nominal_stance_.SetCount(robot_ee_.size());
  nominal_stance_.At(kMapQuadToOpt.at(LF)) = PosXYZ( x_nominal_b,   y_nominal_b, 0.0);
  nominal_stance_.At(kMapQuadToOpt.at(RF)) = PosXYZ( x_nominal_b,  -y_nominal_b, 0.0);
  nominal_stance_.At(kMapQuadToOpt.at(LH)) = PosXYZ(-x_nominal_b,   y_nominal_b, 0.0);
  nominal_stance_.At(kMapQuadToOpt.at(RH)) = PosXYZ(-x_nominal_b,  -y_nominal_b, 0.0);

  II_.SetCount(robot_ee_.size()); II_.SetAll(false);
  PI_.SetCount(robot_ee_.size()); PI_.SetAll(false);
  bI_.SetCount(robot_ee_.size()); bI_.SetAll(false);
  IP_.SetCount(robot_ee_.size()); IP_.SetAll(false);
  Ib_.SetCount(robot_ee_.size()); Ib_.SetAll(false);
  Pb_.SetCount(robot_ee_.size()); Pb_.SetAll(false);
  bP_.SetCount(robot_ee_.size()); bP_.SetAll(false);
  BI_.SetCount(robot_ee_.size()); BI_.SetAll(false);
  IB_.SetCount(robot_ee_.size()); IB_.SetAll(false);
  PP_.SetCount(robot_ee_.size()); PP_.SetAll(false);
  bb_.SetCount(robot_ee_.size()); bb_.SetAll(false);
  Bb_.SetCount(robot_ee_.size()); Bb_.SetAll(false);

  PI_.At(kMapQuadToOpt.at(LH)) = true;
  bI_.At(kMapQuadToOpt.at(RH)) = true;
  IP_.At(kMapQuadToOpt.at(LF)) = true;
  Ib_.At(kMapQuadToOpt.at(RF)) = true;
  Pb_.At(kMapQuadToOpt.at(LH)) = true; Pb_.At(kMapQuadToOpt.at(RF)) = true;
  bP_.At(kMapQuadToOpt.at(RH)) = true; bP_.At(kMapQuadToOpt.at(LF)) = true;
  BI_.At(kMapQuadToOpt.at(LH)) = true; BI_.At(kMapQuadToOpt.at(RH)) = true;
  IB_.At(kMapQuadToOpt.at(LF)) = true; IB_.At(kMapQuadToOpt.at(RF)) = true;
  PP_.At(kMapQuadToOpt.at(LH)) = true; PP_.At(kMapQuadToOpt.at(LF)) = true;
  bb_.At(kMapQuadToOpt.at(RH)) = true; bb_.At(kMapQuadToOpt.at(RF)) = true;
  Bb_.At(kMapQuadToOpt.at(LH)) = true; Bb_.At(kMapQuadToOpt.at(RH)) = true;  Bb_.At(kMapQuadToOpt.at(RF))= true;
}

QuadrupedMotionParameters::MotionTypePtr
QuadrupedMotionParameters::MakeMotion (opt::MotionTypeID id)
{
  switch (id) {
    case WalkID:
      return std::make_shared<Walk>();
    case TrotID:
      return std::make_shared<Trot>();
    case BoundID:
      return std::make_shared<Bound>();
    case PaceID:
      return std::make_shared<Pace>();
    case PushRecID:
      return std::make_shared<PushRecovery>();
    default:
      throw std::runtime_error("MotionTypeID not defined");
  }
}

Walk::Walk()
{
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::WalkID;
  polynomials_per_second_ = 6;

  double t_phase = 0.2;
  double t_trans = 0.1;
//  timings_ =
//  {
//      0.4,
//      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
//      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
//      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
//      0.01,
//  };
//  ee_cycle2_ =
//  {
//      II_,
//      PI_, PP_, IP_, bI_, bb_, Ib_,
//      PI_, PP_, IP_, bI_, bb_, Ib_,
//      PI_, PP_, IP_, bI_, bb_, Ib_,
//      II_,
//  };


  double t_step = 0.4;
  contact_timings_ =
  {
      0.4,
      t_step, t_step,t_step,t_step,
      t_step, t_step,t_step,t_step,
      0.2,
  };

  contact_sequence_ =
  {
      II_,
      PI_, IP_, bI_, Ib_,
      PI_, IP_, bI_, Ib_,
      II_,
  };


  constraints_ = { InitCom,
                   FinalCom,
                   JunctionCom,
                   Convexity,
                   Dynamic,
                   RomBox,
                   Stance,
  };


  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 3.0;
  cost_weights_[PolyCenterCostID]   = 10.0;
//  cost_weights_[FinalComCostID] = 1000.0;
}

Trot::Trot()
{
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::TrotID;
  polynomials_per_second_ = 6;

  double t_phase = 0.3;
  double t_trans = 0.1;

  contact_timings_ =
  {   0.3,
//      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_phase, t_phase, t_phase,
//      0.01
  };

  contact_sequence_ =
  {
      II_,
      //      PI, PP, IP, bI, bb, Ib, // walk
      bP_, Pb_, bP_, Pb_,
//      II_
  };


//    timings_ = {t_phase, t_phase, 0.8 };
//    ee_cycle_ = {bP, Pb, II};
//  timings_ = {0.4,t_phase, t_phase, t_phase, t_phase};
//  ee_cycle_ = {II, bP, Pb, bP, Pb};

  constraints_ = { InitCom,
                   FinalCom,
                   JunctionCom,
                   Dynamic,
                   Convexity,
                   RomBox,
                   Stance,
                   };
  cost_weights_[RangOfMotionCostID] = 10.0;

  // remove all costs hugely speeds up the optimization problem
  cost_weights_[ComCostID]      = 1.0;
//  cost_weights_[FinalComCostID] = 1.0;
//  cost_weights_[PolyCenterCostID]   = 50.0;
}

Pace::Pace()
{
  max_dev_xy_ = {0.20, 0.20};
  id_ = opt::PaceID;
  polynomials_per_second_ = 6;

  contact_timings_ =
  {
      0.3,
      0.3, 0.1, 0.3,
      0.3
  };
  contact_sequence_ =
  {
      II_,
      PP_, II_, bb_,
      II_,
  };

  constraints_ = { InitCom,
                   FinalCom,
                   JunctionCom,
                   Dynamic,
                   Convexity,
                   RomBox,
                   Stance
  };

  cost_weights_[RangOfMotionCostID] = 10.0;
  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Bound::Bound()
{
  max_dev_xy_ = {0.25, 0.25};
  id_ = opt::BoundID;
  polynomials_per_second_ = 6;

  contact_timings_ =
  {
      0.3,
      0.3, 0.1, 0.3,
      0.3
  };
  contact_sequence_ =
  {
      II_,
      BI_, II_, IB_,
      II_
  };


  constraints_ = { InitCom,
                   FinalCom,
                   JunctionCom,
                   Dynamic,
                   Convexity,
                   RomBox,
                   Stance
  };

//  cost_weights_[RangOfMotionCostID] = 10.0;
  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

PushRecovery::PushRecovery ()
{
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::PushRecID;
  polynomials_per_second_ = 5;

  double t_phase = 0.25;
  contact_timings_ = {t_phase, t_phase};
  contact_sequence_ = {bP_, Pb_};

  constraints_ = { InitCom,
//                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[FinalComCostID] = 1000.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

} // namespace quad
} // namespace opt
} // namespace xpp



