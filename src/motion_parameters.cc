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

MotionParameters::PhaseVec
MotionParameters::GetOneCycle () const
{
  PhaseVec phases;
  for (int i=0; i<ee_cycle2_.size(); ++i)
    phases.push_back(Phase(ee_cycle2_.at(i), timings_.at(i)));

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

} // namespace opt
} // namespace xpp

