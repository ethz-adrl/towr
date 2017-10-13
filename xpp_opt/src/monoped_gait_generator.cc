/**
 @file    monoped_gait_generator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#include <../include/xpp_opt/models/monoped_gait_generator.h>

namespace xpp {

MonopedGaitGenerator::MonopedGaitGenerator ()
{
  map_id_to_ee_ = { {"E0", E0 } };
}

MonopedGaitGenerator::GaitInfo
MonopedGaitGenerator::GetGait (GaitTypes gait) const
{
  switch (gait) {
    case Stand:   return GetStrideStand();
    case Flight:  return GetStrideFlight();
    case Walk1:   return GetStrideHop();
    case Walk2:   return GetStrideHop();
    case Run1:    return GetStrideHop();
    case Run2:    return GetStrideHop();
    case Run3:    return GetStrideHop();
    case Hop1:    return GetStrideHop();
    case Hop2:    return GetStrideHop();
    default: assert(false); // gait not implemented
  }
}

MonopedGaitGenerator::GaitInfo
MonopedGaitGenerator::GetStrideStand () const
{
  auto times =
  {
      0.5,
  };
  auto contacts =
  {
      o_,
  };

  return std::make_pair(times, contacts);
}

MonopedGaitGenerator::GaitInfo
MonopedGaitGenerator::GetStrideFlight () const
{
  auto times =
  {
      0.5,
  };
  auto contacts =
  {
      x_,
  };

  return std::make_pair(times, contacts);
}

MonopedGaitGenerator::GaitInfo
MonopedGaitGenerator::GetStrideHop () const
{
  auto times =
  {
      0.3,
      0.3,
  };
  auto contacts =
  {
      o_,
      x_,
  };

  return std::make_pair(times, contacts);
}

MonopedGaitGenerator::~MonopedGaitGenerator ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace xpp */
