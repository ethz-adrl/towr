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
  max_dev_xy_ = {0.15, 0.15};
  weight_com_motion_xy_ = {1.0, 1.0};
  start_with_stance_ = true;
  walking_height_ = 0.58;
  lift_height_ = 0.12;

  constraints_ = { InitCom,
//                   FinalCom,
//                   FinalStance,
                   JunctionCom,
                   Convexity,
                   SuppArea,
                   Dynamic,
                   RomBox};

  cost_weights_[FinalComCostID] = 100.0;
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
  id_ = opt::WalkID;
  t_phase_ = 0.5;
  max_step_length_ = 0.21;
  dt_nodes_ = 0.05;
  polynomials_per_phase_ = 1;

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 1.0;
  cost_weights_[PolyCenterCostID]   = 10.0;
}

Trott::Trott()
{
  id_ = opt::TrottID;
  t_phase_ = 0.3;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.05;
  polynomials_per_phase_ = 1;

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

PushRecovery::PushRecovery ()
{
  id_ = opt::PushRecID;
  start_with_stance_ = false;
  t_phase_ = 0.2;
  max_step_length_ = 0.35;
  dt_nodes_ = 0.05;
  polynomials_per_phase_ = 1;
  walking_height_ = 0.55;
  lift_height_ = 0.08;

  auto &v = constraints_;
  v.erase(std::remove(v.begin(), v.end(), FinalCom), v.end());
  v.erase(std::remove(v.begin(), v.end(), FinalStance), v.end());

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Camel::Camel()
{
  id_ = opt::CamelID;
  t_phase_ = 0.3;
  max_step_length_ = 0.25;
  dt_nodes_ = 0.03;
  polynomials_per_phase_ = 3;

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Bound::Bound()
{
  id_ = opt::BoundID;
  t_phase_ = 0.3;
  max_step_length_ = 0.4;
  dt_nodes_ = 0.04;
  polynomials_per_phase_ = 2;

  cost_weights_[ComCostID]          = 1.0;
  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Walk::SwingLegCycle
Walk::GetOneCycle () const
{
  SwingLegCycle cycle;
  cycle.push_back({kMapHyqToOpt.at(LH)});
  cycle.push_back({kMapHyqToOpt.at(LF)});
  cycle.push_back({kMapHyqToOpt.at(RH)});
  cycle.push_back({kMapHyqToOpt.at(RF)});
  return cycle;
}

Trott::SwingLegCycle
Trott::GetOneCycle () const
{
  SwingLegCycle cycle;
  cycle.push_back({kMapHyqToOpt.at(LF), kMapHyqToOpt.at(RH)});
  cycle.push_back({kMapHyqToOpt.at(RF), kMapHyqToOpt.at(LH)});
  return cycle;
}

PushRecovery::SwingLegCycle
PushRecovery::GetOneCycle () const
{
  SwingLegCycle cycle;
  cycle.push_back({kMapHyqToOpt.at(LF), kMapHyqToOpt.at(RH)});
  cycle.push_back({kMapHyqToOpt.at(RF), kMapHyqToOpt.at(LH)});
  return cycle;
}

Camel::SwingLegCycle
Camel::GetOneCycle () const
{
  SwingLegCycle cycle;
  cycle.push_back({kMapHyqToOpt.at(LH), kMapHyqToOpt.at(LF)});
  cycle.push_back({kMapHyqToOpt.at(RH), kMapHyqToOpt.at(RF)});
  return cycle;
}

Bound::SwingLegCycle
Bound::GetOneCycle () const
{
  SwingLegCycle cycle;
  cycle.push_back({kMapHyqToOpt.at(LF), kMapHyqToOpt.at(RF)});
  cycle.push_back({kMapHyqToOpt.at(LH), kMapHyqToOpt.at(RH)});
  return cycle;
}

} // namespace hyq
} // namespace xpp



