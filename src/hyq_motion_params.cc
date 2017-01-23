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

HyqMotionParameters::HyqMotionParameters ()
{
  lambda_deviation_percent_ = 1.0; // 100 percent
  weight_com_motion_xy_ = {1.0, 1.0};
  start_with_stance_ = true;
  walking_height_ = 0.58;
  lift_height_ = 0.08;
}

HyqMotionParameters::NominalStance
HyqMotionParameters::GetNominalStanceInBase () const
{
  const double x_nominal_b = 0.34;
  const double y_nominal_b = 0.34;

  NominalStance nominal;
  nominal[kMapHyqToOpt.at(LF)] = PosXY( x_nominal_b,   y_nominal_b);
  nominal[kMapHyqToOpt.at(RF)] = PosXY( x_nominal_b,  -y_nominal_b);
  nominal[kMapHyqToOpt.at(LH)] = PosXY(-x_nominal_b,   y_nominal_b);
  nominal[kMapHyqToOpt.at(RH)] = PosXY(-x_nominal_b,  -y_nominal_b);

  return nominal;
}

Walk::Walk()
{
  opt_horizon_in_phases_ = 12;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::WalkID;
  t_phase_ = 0.4;
  max_step_length_ = 0.21;
  dt_nodes_ = 0.1;
  polynomials_per_second_ = 3;


  constraints_ = { InitCom,
                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};


  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 1.0;
  cost_weights_[PolyCenterCostID]   = 10.0;
//  cost_weights_[FinalComCostID] = 1000.0;
}

Trott::Trott()
{
  opt_horizon_in_phases_ = 2;
  max_dev_xy_ = {0.15, 0.15};
  lambda_deviation_percent_ = 0.6;
  id_ = opt::TrottID;
  start_with_stance_ = false;
  t_phase_ = 0.3;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.05;
  polynomials_per_second_ = 3;

  constraints_ = { InitCom,
                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox
                   };

  // remove all costs hugely speeds up the optimization problem
//  cost_weights_[ComCostID]      = 1.0;
//  cost_weights_[FinalComCostID] = 1.0;
//  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

PushRecovery::PushRecovery ()
{
  opt_horizon_in_phases_ = 2;
  max_dev_xy_ = {0.15, 0.15};
  lambda_deviation_percent_ = 0.8;
  id_ = opt::PushRecID;
  start_with_stance_ = false;
  t_phase_ = 0.2;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.1;
  polynomials_per_second_ = 5;
  walking_height_ = 0.55;
  lift_height_ = 0.08;

  constraints_ = { InitCom,
//                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};

//  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[FinalComCostID] = 1000.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Camel::Camel()
{
  opt_horizon_in_phases_ = 2;
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::CamelID;
  t_phase_ = 0.3;
  max_step_length_ = 0.25;
  dt_nodes_ = 0.03;
  polynomials_per_second_ = 10;

  constraints_ = { InitCom,
                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Bound::Bound()
{
  max_dev_xy_ = {0.15, 0.15};
  id_ = opt::BoundID;
  t_phase_ = 0.3;
  max_step_length_ = 0.4;
  dt_nodes_ = 0.03;
  polynomials_per_second_ = 10;


  constraints_ = { InitCom,
                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

// naming convention:, where the circle is is a swingleg, front is right ->.
// so LF and RH swinging is (bP):  o x
//                                 x o
static const MotionParameters::Swinglegs II = {};
static const MotionParameters::Swinglegs PI = {kMapHyqToOpt.at(LH)};
static const MotionParameters::Swinglegs bI = {kMapHyqToOpt.at(RH)};
static const MotionParameters::Swinglegs IP = {kMapHyqToOpt.at(LF)};
static const MotionParameters::Swinglegs Ib = {kMapHyqToOpt.at(RF)};
static const MotionParameters::Swinglegs Pb = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(RF)};
static const MotionParameters::Swinglegs bP = {kMapHyqToOpt.at(RH), kMapHyqToOpt.at(LF)};
static const MotionParameters::Swinglegs BI = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(RH)};
static const MotionParameters::Swinglegs IB = {kMapHyqToOpt.at(LF), kMapHyqToOpt.at(RF)};
static const MotionParameters::Swinglegs PP = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(LF)};
static const MotionParameters::Swinglegs bb = {kMapHyqToOpt.at(RH), kMapHyqToOpt.at(RF)};
static const MotionParameters::Swinglegs Bb = {kMapHyqToOpt.at(LH), kMapHyqToOpt.at(RH), kMapHyqToOpt.at(RF)};


Walk::SwingLegCycle
Walk::GetOneCycle () const
{
  return {PI, IP, bI, Ib};
}

Trott::SwingLegCycle
Trott::GetOneCycle () const
{
  return {bP, Pb};
}

PushRecovery::SwingLegCycle
PushRecovery::GetOneCycle () const
{
  return {bP, Pb};
}

Camel::SwingLegCycle
Camel::GetOneCycle () const
{
  return {PP, bb};
}

Bound::SwingLegCycle
Bound::GetOneCycle () const
{
  return {BI, IB};
}

} // namespace hyq
} // namespace xpp



