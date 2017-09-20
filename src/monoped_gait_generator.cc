/**
 @file    monoped_gait_generator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#include <xpp/models/monoped_gait_generator.h>

namespace xpp {
namespace opt {

MonopedGaitGenerator::MonopedGaitGenerator ()
{
  map_id_to_ee_ = { {"E0", E0 } };
}

MonopedGaitGenerator::GaitInfo
MonopedGaitGenerator::GetGait (opt::GaitTypes gait) const
{
  switch (gait) {
    case opt::Stand:   return GetStrideStand();
    case opt::Flight:  return GetStrideFlight();
    case opt::Walk1:   return GetStrideHop();
    case opt::Walk2:   return GetStrideHop();
    case opt::Run1:    return GetStrideHop();
    case opt::Run2:    return GetStrideHop();
    case opt::Run3:    return GetStrideHop();
    case opt::Hop1:    return GetStrideHop();
    case opt::Hop2:    return GetStrideHop();
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

} /* namespace opt */
} /* namespace xpp */
