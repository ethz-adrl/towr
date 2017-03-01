/**
 @file    hyq_motion_params.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Brief description
 */

#include <xpp/hyq/hyq_motion_params.h>
#include <xpp/hyq/ee_hyq.h>
#include <algorithm>

namespace xpp {
namespace hyq {

using namespace xpp::opt;

// naming convention:, where the circle is is a swingleg, front is right ->.
// so LF and RH swinging is (bP):  o x
//                                 x o
static const MotionParameters::EEIDVec II = {};
static const MotionParameters::EEIDVec PI = {kMapHyqToOpt.at(LH)};
static const MotionParameters::EEIDVec bI = {kMapHyqToOpt.at(RH)};
static const MotionParameters::EEIDVec IP = {kMapHyqToOpt.at(LF)};
static const MotionParameters::EEIDVec Ib = {kMapHyqToOpt.at(RF)};
static const MotionParameters::EEIDVec Pb = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(RF)};
static const MotionParameters::EEIDVec bP = {kMapHyqToOpt.at(RH), kMapHyqToOpt.at(LF)};
static const MotionParameters::EEIDVec BI = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(RH)};
static const MotionParameters::EEIDVec IB = {kMapHyqToOpt.at(LF), kMapHyqToOpt.at(RF)};
static const MotionParameters::EEIDVec PP = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(LF)};
static const MotionParameters::EEIDVec bb = {kMapHyqToOpt.at(RH), kMapHyqToOpt.at(RF)};
static const MotionParameters::EEIDVec Bb = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(RH), kMapHyqToOpt.at(RF)};


HyqMotionParameters::HyqMotionParameters ()
{
  lambda_deviation_percent_ = 1.0; // 100 percent
  weight_com_motion_xy_ = {1.0, 1.0};
  geom_walking_height_ = 0.58;
  lift_height_ = 0.08;
//  offset_geom_to_com_ << -0.02230, -0.00010, 0.03870;
//  offset_geom_to_com_ << -0.03, 0.02, 0.0;
  offset_geom_to_com_ << 0,0,0;
  robot_ee_ = { EEID::E0, EEID::E1, EEID::E2, EEID::E3 };


  const double x_nominal_b = 0.34;
  const double y_nominal_b = 0.34;
  NominalStance nominal;
  nominal_stance_[kMapHyqToOpt.at(LF)] = PosXY( x_nominal_b,   y_nominal_b);
  nominal_stance_[kMapHyqToOpt.at(RF)] = PosXY( x_nominal_b,  -y_nominal_b);
  nominal_stance_[kMapHyqToOpt.at(LH)] = PosXY(-x_nominal_b,   y_nominal_b);
  nominal_stance_[kMapHyqToOpt.at(RH)] = PosXY(-x_nominal_b,  -y_nominal_b);
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

  double t_phase = 0.4;
  timings_ = {t_phase, t_phase, t_phase, t_phase};
  ee_cycle_ = {PI, IP, bI, Ib};

//  double swing = 0.4;
//  double trans = 0.1;
//  timings_ = {swing, trans, swing, trans, swing, trans, swing, trans};
//  ee_cycle_ = {PI, PP, IP, bP, bI, bb, Ib, Pb};


  constraints_ = { InitCom,
                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};


  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 3.0;
  cost_weights_[PolyCenterCostID]   = 50.0;
//  cost_weights_[FinalComCostID] = 1000.0;
}

Trott::Trott()
{
  opt_horizon_in_phases_ = 2*5;//2*2;
  max_dev_xy_ = {0.15, 0.15};
//  lambda_deviation_percent_ = 0.6;
  id_ = opt::TrottID;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.05;
  polynomials_per_second_ = 3;

  double t_phase = 0.3;
  timings_ = {t_phase, t_phase};
  ee_cycle_ = {bP, Pb};

  constraints_ = { InitCom,
                   FinalCom,
                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox
                   };

  // remove all costs hugely speeds up the optimization problem
//  cost_weights_[ComCostID]      = 1.0;
//  cost_weights_[FinalComCostID] = 1.0;
  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

PushRecovery::PushRecovery ()
{
  opt_horizon_in_phases_ = 2*3;
  max_dev_xy_ = {0.15, 0.15};
  lambda_deviation_percent_ = 0.8;
  id_ = opt::PushRecID;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.1;
  polynomials_per_second_ = 5;
  geom_walking_height_ = 0.58;
  lift_height_ = 0.08;

  double t_phase = 0.25;
  timings_ = {t_phase, t_phase};
  ee_cycle_ = {bP, Pb};

  constraints_ = { InitCom,
//                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[FinalComCostID] = 1000.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Pace::Pace()
{
  opt_horizon_in_phases_ = 4*1;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::PaceID;
  max_step_length_ = 0.25;
  dt_nodes_ = 0.02;
  polynomials_per_second_ = 20;

  timings_ = {0.1, 0.3, 0.1, 0.3};
  ee_cycle_ = {II, PP, II, bb};

  constraints_ = { InitCom,
//                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Bound::Bound()
{
  opt_horizon_in_phases_ = 4*1;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::BoundID;
  max_step_length_ = 0.4;
  dt_nodes_ = 0.02;
  polynomials_per_second_ = 20;

  timings_ = {0.1, 0.3, 0.1, 0.3};
  ee_cycle_ = {II, BI, II, IB};


  constraints_ = { InitCom,
//                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

} // namespace hyq
} // namespace xpp



