/**
@file    motion_type.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#include <xpp/opt/motion_type.h>

namespace xpp {
namespace opt {

MotionType::~MotionType ()
{
}

MotionType::MotionTypePtr
MotionType::MakeMotion (MotionTypeID id)
{
  switch (id) {
    case WalkID:
      return std::make_shared<hyq::Walk>();
    case TrottID:
      return std::make_shared<hyq::Trott>();
    case opt::BoundID:
      return std::make_shared<hyq::Bound>();
    case opt::CamelID:
      return std::make_shared<hyq::Camel>();
    default:
      throw std::runtime_error("ID not defined");
  }
}

} // namespace opt
} // namespace xpp


#include <xpp/hyq/ee_hyq.h>

namespace xpp {
namespace hyq {

using LegIDVec = std::vector<hyq::LegID>;

Walk::Walk()
{
  id_ = opt::WalkID;
  t_phase_ = 0.5;
  max_step_length_ = 0.21;

  weight_com_motion_cost_      = 1.0;
  weight_range_of_motion_cost_ = 1.0;
  weight_polygon_center_cost_  = 10.0;
}

Trott::Trott()
{
  id_ = opt::TrottID;
  t_phase_ = 0.3;
  max_step_length_ = 0.15;

  weight_com_motion_cost_      = 1.0;
  weight_range_of_motion_cost_ = 1.0;
  weight_polygon_center_cost_  = 10.0;
}

Camel::Camel()
{
  id_ = opt::CamelID;
  t_phase_ = 0.3;
  max_step_length_ = 0.15;

  weight_com_motion_cost_      = 1.0;
  weight_range_of_motion_cost_ = 100.0;
  weight_polygon_center_cost_  = 0.0;
}

Bound::Bound()
{
  id_ = opt::BoundID;
  t_phase_ = 0.3;
  max_step_length_ = 0.15;

  weight_com_motion_cost_      = 1.0;
  weight_range_of_motion_cost_ = 100.0;
  weight_polygon_center_cost_  = 0.0;
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

} // namespace opt
} // namespace xpp

