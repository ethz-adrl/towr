/**
 @file    biped_gait_generator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#include <xpp/models/biped_gait_generator.h>

namespace xpp {
namespace opt {

BipedGaitGenerator::BipedGaitGenerator ()
{
  ContactState init(2, false);
  I_ = b_ = P_ = B_ = init;

  map_id_to_ee_ = biped::kMapIDToEE;
  I_.SetAll(false);
  P_.At(map_id_to_ee_.at(biped::L)) = true;
  b_.At(map_id_to_ee_.at(biped::R)) = true;
  B_.SetAll(true);

  SetGaits({Stand});
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetGait (GaitTypes gait) const
{
  switch (gait) {
    case Stand:   return GetStrideStand();
    case Flight:  return GetStrideFlight();
    case Walk1:   return GetStrideWalk();
    case Walk2:   return GetStrideWalk();
    case Run1:    return GetStrideRun();
    case Run2:    return GetStrideRun();
    case Run3:    return GetStrideRun();
    case Hop1:    return GetStrideHop();
    case Hop2:    return GetStrideHop();
    default: assert(false); // gait not implemented
  }
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideStand () const
{
  auto times =
  {
      0.5,
  };
  auto contacts =
  {
      B_,
  };

  return std::make_pair(times, contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideFlight () const
{
  auto times =
  {
      0.5,
  };
  auto contacts =
  {
      I_,
  };

  return std::make_pair(times, contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideWalk () const
{
  double step = 0.3;
  double stance = 0.1;
  auto times =
  {
      step, stance,
      step, stance,
  };
  auto phase_contacts =
  {
      b_, B_, // swing left foot
      P_, B_, // swing right foot
  };

  return std::make_pair(times, phase_contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideRun () const
{
  double step = 0.3;
  double flight = 0.2;
  auto times =
  {
      step, flight,
      step, flight,
  };
  auto phase_contacts =
  {
      b_, I_, // swing left foot
      P_, I_, // swing right foot
  };

  return std::make_pair(times, phase_contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideHop () const
{
  double push = 0.3;
  double flight = 0.2;
  auto times =
  {
      push, flight,
  };
  auto phase_contacts =
  {
      B_, I_,
  };

  return std::make_pair(times, phase_contacts);
}

BipedGaitGenerator::~BipedGaitGenerator ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace opt */
} /* namespace xpp */
