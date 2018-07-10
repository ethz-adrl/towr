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

#include <towr/initialization/quadruped_gait_generator.h>

#include <cassert>
#include <iostream>

#include <towr/models/endeffector_mappings.h>

namespace towr {

QuadrupedGaitGenerator::QuadrupedGaitGenerator ()
{
  int n_ee = 4;
  ContactState init(n_ee, false);

  II_                               = init; // flight_phase
  PI_ = bI_ = IP_ = Ib_             = init; // single contact
  Pb_ = bP_ = BI_ = IB_ = PP_ = bb_ = init; // two leg support
  Bb_ = BP_ = bB_ = PB_             = init; // three-leg support
  BB_                               = init; // four-leg support phase

  // flight_phase
  II_ = ContactState(n_ee, false);
  // one stanceleg
  PI_.at(LH) = true;
  bI_.at(RH) = true;
  IP_.at(LF) = true;
  Ib_.at(RF) = true;
  // two stancelegs
  Pb_.at(LH) = true; Pb_.at(RF) = true;
  bP_.at(RH) = true; bP_.at(LF) = true;
  BI_.at(LH) = true; BI_.at(RH) = true;
  IB_.at(LF) = true; IB_.at(RF) = true;
  PP_.at(LH) = true; PP_.at(LF) = true;
  bb_.at(RH) = true; bb_.at(RF) = true;
  // three stancelegs
  Bb_.at(LH) = true; Bb_.at(RH) = true;  Bb_.at(RF)= true;
  BP_.at(LH) = true; BP_.at(RH) = true;  BP_.at(LF)= true;
  bB_.at(RH) = true; bB_.at(LF) = true;  bB_.at(RF)= true;
  PB_.at(LH) = true; PB_.at(LF) = true;  PB_.at(RF)= true;
  // four stancelgs
  BB_ = ContactState(n_ee, true);

  // default gait
  SetGaits({Stand});
}

void
QuadrupedGaitGenerator::SetCombo (Combos combo)
{
  switch (combo) {
    case C0: SetGaits({Stand, Walk2, Walk2, Walk2, Walk2E, Stand}); break; // overlap-walk
    case C1: SetGaits({Stand, Run2, Run2, Run2, Run2E, Stand});     break; // fly trot
    case C2: SetGaits({Stand, Run3, Run3, Run3, Run3E, Stand}); break; // pace
    case C3: SetGaits({Stand, Hop1, Hop1, Hop1, Hop1E, Stand}); break; // bound
    case C4: SetGaits({Stand, Hop3, Hop3, Hop3, Hop3E, Stand}); break; // gallop
    default: assert(false); std::cout << "Gait not defined\n"; break;
  }
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetGait(Gaits gait) const
{
  switch (gait) {
    case Stand:   return GetStrideStand();
    case Flight:  return GetStrideFlight();
    case Walk1:   return GetStrideWalk();
    case Walk2:   return GetStrideWalkOverlap();
    case Walk2E:  return RemoveTransition(GetStrideWalkOverlap());
    case Run1:    return GetStrideTrot();
    case Run2:    return GetStrideTrotFly();
    case Run2E:   return GetStrideTrotFlyEnd();
    case Run3:    return GetStridePace();
    case Run3E:   return GetStridePaceEnd();
    case Hop1:    return GetStrideBound();
    case Hop1E:   return GetStrideBoundEnd();
    case Hop2:    return GetStridePronk();
    case Hop3:    return GetStrideGallop();
    case Hop3E:   return RemoveTransition(GetStrideGallop());
    case Hop5:    return GetStrideLimp();
    default: assert(false); // gait not implemented
  }
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideStand () const
{
  auto times =
  {
      0.3,
  };
  auto contacts =
  {
      BB_,
  };

  return std::make_pair(times, contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideFlight () const
{
  auto times =
  {
      0.3,
  };
  auto contacts =
  {
      Bb_,
  };

  return std::make_pair(times, contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStridePronk () const
{
  double push   = 0.3;
  double flight = 0.4;
  double land   = 0.3;

  auto times =
  {
      push, flight, land
  };
  auto phase_contacts =
  {
      BB_, II_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideWalk () const
{
  double step  = 0.3;
  double stand = 0.2;
  auto times =
  {
      step, stand, step, stand,
      step, stand, step, stand,
  };
  auto phase_contacts =
  {
      bB_, BB_, Bb_, BB_,
      PB_, BB_, BP_, BB_
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideWalkOverlap () const
{
  double three    = 0.25;
  double lateral  = 0.13;
  double diagonal = 0.13;

  auto times =
  {
      three, lateral, three,
      diagonal,
      three, lateral, three,
      diagonal,
  };
  auto phase_contacts =
  {
      bB_, bb_, Bb_,
      Pb_, // start lifting RH
      PB_, PP_, BP_,
      bP_, // start lifting LH
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideTrot () const
{
  double t_step = 0.3;
  double t_stand = 0.2;
  auto times =
  {
      t_step, t_stand, t_step, t_stand,
  };
  auto phase_contacts =
  {
      bP_, BB_, Pb_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideTrotFly () const
{
  double stand   = 0.4;
  double flight = 0.1;
  auto times =
  {
      stand, flight,
      stand, flight,
  };
  auto phase_contacts =
  {
      bP_, II_,
      Pb_, II_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideTrotFlyEnd () const
{
  auto times =
  {
      0.4,
  };
  auto phase_contacts =
  {
      bP_
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStridePace () const
{
  double stand  = 0.3;
  double flight = 0.1;

  auto times =
  {
      stand, flight, stand, flight
  };
  auto phase_contacts =
  {
      PP_, II_, bb_, II_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStridePaceEnd () const
{
  auto times =
  {
      0.3,
  };
  auto phase_contacts =
  {
      PP_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideBound () const
{
  double stand  = 0.3;
  double flight = 0.1;

  auto times =
  {
      stand, flight, stand, flight
  };
  auto phase_contacts =
  {
      BI_, II_, IB_, II_
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideBoundEnd () const
{
  auto times =
  {
      0.3,
  };
  auto phase_contacts =
  {
      BI_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideGallop () const
{
  double A = 0.3; // both feet in air
  double B = 0.2; // overlap
  double C = 0.2; // transition front->hind
  auto times =
  {
      B, A, B,
      C,
      B, A, B,
      C
  };
  auto phase_contacts =
  {
      Bb_, BI_, BP_,  // front legs swing forward
      bP_,            // transition phase
      bB_, IB_, PB_,  // hind legs swing forward
      Pb_
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideLimp () const
{
  double A = 0.1; // three in contact
  double B = 0.2; // all in contact
  double C = 0.1; // one in contact

  auto times =
  {
      A, B, C,
      A, B, C,
  };
  auto phase_contacts =
  {
      Bb_, BB_, IP_,
      Bb_, BB_, IP_,
  };

  return std::make_pair(times, phase_contacts);
}

} /* namespace towr */
