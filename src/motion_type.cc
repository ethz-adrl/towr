/**
@file    motion_type.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#include <xpp/opt/motion_parameters.h>
#include <xpp/hyq/hyq_motion_params.h>

namespace xpp {
namespace opt {

MotionParameters::~MotionParameters ()
{
}

MotionParameters::MotionTypePtr
MotionParameters::MakeMotion (MotionTypeID id)
{
  switch (id) {
    case WalkID:
      return std::make_shared<hyq::Walk>();
    case TrottID:
      return std::make_shared<hyq::Trott>();
    case BoundID:
      return std::make_shared<hyq::Bound>();
    case CamelID:
      return std::make_shared<hyq::Camel>();
    default:
      throw std::runtime_error("MotionTypeID not defined");
  }
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

} // namespace opt
} // namespace xpp

