/**
@file    motion_type.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#include <xpp/opt/motion_parameters.h>

namespace xpp {
namespace opt {

MotionParameters::~MotionParameters ()
{
}

MotionParameters::SwinglegPhaseVec
MotionParameters::GetOneCycle () const
{
  SwinglegPhaseVec phases;
  for (int i=0; i<ee_cycle_.size(); ++i)
    phases.push_back(SwinglegPhase(ee_cycle_.at(i), timings_.at(i)));

  return phases;
}

MotionParameters::ValXY
MotionParameters::GetMaximumDeviationFromNominal () const
{
  return max_dev_xy_;
}

MotionParameters::CostWeights
MotionParameters::GetCostWeights () const
{
  return cost_weights_;
}

MotionParameters::UsedConstraints
MotionParameters::GetUsedConstraints () const
{
  return constraints_;
}

// zmp_ use this instead of com->GetTotalTime()
double
MotionParameters::GetTotalTime () const
{
  double T = 0.0;
  for (int i = 0; i<opt_horizon_in_phases_; ++i) {
    int k = i%ee_cycle_.size();
    T += timings_.at(k);
  }

  T += 0.4 + 0.8; // zmp_ HORRIBLE! remove, initial and final stance

  return T;
}

} // namespace opt
} // namespace xpp

