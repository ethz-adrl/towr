/**
 @file    biped_gait_generator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 20, 2017
 @brief   Brief description
 */

#include <towr/models/biped_gait_generator.h>

#include <cassert>
#include <iostream>

#include <towr/models/endeffector_mappings.h>

namespace towr {

BipedGaitGenerator::BipedGaitGenerator ()
{
  ContactState init(2, false);
  I_ = b_ = P_ = B_ = init;

  using namespace biped;
  P_.at(L) = true;
  b_.at(R) = true;
  B_       = { true, true };

  SetGaits({Stand});
}

void
BipedGaitGenerator::SetCombo (GaitCombos combo)
{
  switch (combo) {
    case Combo0: SetGaits({Stand});                                break;
    case Combo1: SetGaits({Stand, Flight, Stand});                 break;
    case Combo2: SetGaits({Stand, Walk1, Walk1, Stand});           break;
    case Combo3: SetGaits({Stand, Run1, Run1, Stand});             break;
    case Combo4: SetGaits({Stand, Hop1, Hop1, Stand});             break;
    case Combo5: SetGaits({Stand, Hop2, Hop2, Hop2, Stand});       break;
    case Combo6: SetGaits({Stand, Hop3, Hop3, Hop3, Hop3, Stand}); break;
//    case Combo7: SetGaits({Stand,
//                           Run1, Run1, Run1,
//                           Run1, Run1,
//                           Stand});
//      break;
    case Combo7: SetGaits({Stand,
                           Walk1, Walk1, Walk1,
                           Walk1, Walk1,
                           Stand});
      break;
    // for RA-L sequence video
    case Combo8: SetGaits({Stand,
                           Walk1, Walk1,
                           Run1, Run1, Run1,
                           Hop2, Hop2, Hop2, Hop2,
                           Stand, Stand,
                           Hop5, Hop5,
                           Hop1,
                           Stand,
                          });
      break;
    default: assert(false); std::cout << "Gait not defined\n";     break;
  }
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
    case Run3:    return GetStrideRun();
    case Hop1:    return GetStrideHop();
    case Hop2:    return GetStrideLeftHop();
    case Hop3:    return GetStrideRightHop();
    case Hop5:    return GetStrideGallopHop();
    default: assert(false); // gait not implemented
  }
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideStand () const
{
  auto times =
  {
      0.2,
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
  double stance = 0.05;
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
//  double step = 0.3;
  double flight = 0.4;
  double pushoff = 0.15;
  double landing = 0.15;
  auto times =
  {
      pushoff, flight,
      landing+pushoff, flight, landing,
  };
  auto phase_contacts =
  {
      b_, I_,     // swing left foot
      P_, I_, b_, // swing right foot
  };

  return std::make_pair(times, phase_contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideHop () const
{
  double push   = 0.15;
  double flight = 0.5;
  double land   = 0.15;
  auto times =
  {
      push, flight, land
  };
  auto phase_contacts =
  {
      B_, I_, B_,
  };

  return std::make_pair(times, phase_contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideGallopHop () const
{
  double push   = 0.2;
  double flight = 0.3;
  double land   = 0.2;

  auto times =
  {
      push, flight,
      land, land,
  };
  auto phase_contacts =
  {
      P_, I_,
      b_, B_,
  };

  return std::make_pair(times, phase_contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideLeftHop () const
{
  double push   = 0.15;
  double flight = 0.4;
  double land   = 0.15;

  auto times =
  {
      push, flight, land,
  };
  auto phase_contacts =
  {
      b_, I_, b_
  };

  return std::make_pair(times, phase_contacts);
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetStrideRightHop () const
{
  double push   = 0.2;
  double flight = 0.2;
  double land   = 0.2;

  auto times =
  {
      push, flight, land
  };
  auto phase_contacts =
  {
      P_, I_, P_
  };

  return std::make_pair(times, phase_contacts);
}

} /* namespace towr */
