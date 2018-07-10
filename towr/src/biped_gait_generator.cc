/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/initialization/biped_gait_generator.h>

#include <cassert>
#include <iostream>

#include <towr/models/endeffector_mappings.h>

namespace towr {

BipedGaitGenerator::BipedGaitGenerator ()
{
  ContactState init(2, false);
  I_ = b_ = P_ = B_ = init;

  P_.at(L) = true;
  b_.at(R) = true;
  B_       = { true, true };

  SetGaits({Stand});
}

void
BipedGaitGenerator::SetCombo (Combos combo)
{
  switch (combo) {
    case C0: SetGaits({Stand, Walk1, Walk1, Walk1, Walk1, Stand}); break;
    case C1: SetGaits({Stand, Run1, Run1, Run1, Run1, Stand});     break;
    case C2: SetGaits({Stand, Hop1, Hop1, Hop1, Stand});       break;
    case C3: SetGaits({Stand, Hop1, Hop2, Hop2, Stand});       break;
    case C4: SetGaits({Stand, Hop5, Hop5, Hop5, Stand});       break;
    default: assert(false); std::cout << "Gait not defined\n"; break;
  }
}

BipedGaitGenerator::GaitInfo
BipedGaitGenerator::GetGait (Gaits gait) const
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
