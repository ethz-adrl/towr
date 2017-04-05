/**
 @file    hyq_motion_params.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Brief description
 */

#include <xpp/endeffectors4.h>
#include <xpp/opt/quadruped_motion_parameters.h>

namespace xpp {
namespace opt {


QuadrupedMotionParameters::QuadrupedMotionParameters ()
{
  weight_com_motion_xy_ = {1.0, 1.0};
  geom_walking_height_ = 0.58;
  lift_height_ = 0.08;
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
    case TrottID:
      return std::make_shared<Trott>();
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
//  opt_horizon_in_phases_ = 4*1.5;
  opt_horizon_in_phases_ = 16;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::WalkID;
  max_step_length_ = 0.21;
  dt_nodes_ = 0.1;
  polynomials_per_second_ = 3;

  double t_phase = 0.2;
  double t_trans = 0.1;
  timings_ =
  {
      0.4,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      0.01,
  };
  ee_cycle2_ =
  {
      II_,
      PI_, PP_, IP_, bI_, bb_, Ib_,
      PI_, PP_, IP_, bI_, bb_, Ib_,
      PI_, PP_, IP_, bI_, bb_, Ib_,
      II_,
  };

//  double swing = 0.4;
//  double trans = 0.1;
//  timings_ = {swing, trans, swing, trans, swing, trans, swing, trans};
//  ee_cycle_ = {PI, PP, IP, bP, bI, bb, Ib, Pb};


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
  cost_weights_[PolyCenterCostID]   = 50.0;
//  cost_weights_[FinalComCostID] = 1000.0;
}

Trott::Trott()
{
  opt_horizon_in_phases_ = 2*2;
  max_dev_xy_ = {0.15, 0.15};
//  lambda_deviation_percent_ = 0.6;
  id_ = opt::TrottID;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.05;
  polynomials_per_second_ = 15;

  double t_phase = 0.3;
  double t_trans = 0.1;

  timings_ =
  {   0.3,
//      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_phase, t_phase, t_phase,
      0.01
  };

  ee_cycle2_ =
  {
      II_,
      //      PI, PP, IP, bI, bb, Ib, // walk
      bP_, Pb_, bP_, Pb_,
      II_
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

PushRecovery::PushRecovery ()
{
  opt_horizon_in_phases_ = 2*3;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::PushRecID;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.1;
  polynomials_per_second_ = 5;
  geom_walking_height_ = 0.58;
  lift_height_ = 0.08;

  double t_phase = 0.25;
  timings_ = {t_phase, t_phase};
  ee_cycle2_ = {bP_, Pb_};

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

Pace::Pace()
{
  opt_horizon_in_phases_ = 4*4;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::PaceID;
  max_step_length_ = 0.25;
  dt_nodes_ = 0.02;
  polynomials_per_second_ = 20;

  timings_ = {0.1, 0.3, 0.1, 0.3};
  ee_cycle2_ = {II_, PP_, II_, bb_};

  constraints_ = { InitCom,
                   FinalCom,
                   Stance,
                   JunctionCom,
                   Convexity,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Bound::Bound()
{
  opt_horizon_in_phases_ = 4*4;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::BoundID;
  max_step_length_ = 0.4;
  dt_nodes_ = 0.02;
  polynomials_per_second_ = 20;

  timings_ = {0.1, 0.3, 0.1, 0.3};
  ee_cycle2_ = {II_, BI_, II_, IB_};


  constraints_ = { InitCom,
                   FinalCom,
                   Stance,
                   JunctionCom,
                   Convexity,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

} // namespace opt
} // namespace xpp



