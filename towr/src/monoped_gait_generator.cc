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

#include <towr/initialization/monoped_gait_generator.h>

#include <cassert>
#include <iostream>

namespace towr {

void
MonopedGaitGenerator::SetCombo (Combos combo)
{
  switch (combo) {
    case C0: SetGaits({Stand, Hop1, Hop1, Hop1, Hop1, Stand});       break;
    case C1: SetGaits({Stand, Hop1, Hop1, Hop1, Stand});             break;
    case C2: SetGaits({Stand, Hop1, Hop1, Hop1, Hop1, Stand});       break;
    case C3: SetGaits({Stand, Hop2, Hop2, Hop2, Stand});             break;
    case C4: SetGaits({Stand, Hop2, Hop2, Hop2, Hop2, Hop2, Stand}); break;
    default: assert(false); std::cout << "Gait not defined\n";       break;
  }
}

MonopedGaitGenerator::GaitInfo
MonopedGaitGenerator::GetGait (Gaits gait) const
{
  switch (gait) {
    case Stand:   return GetStrideStand();
    case Flight:  return GetStrideFlight();
    case Hop1:    return GetStrideHop();
    case Hop2:    return GetStrideHopLong();
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
      0.3, 0.3,
  };
  auto contacts =
  {
      o_, x_,
  };

  return std::make_pair(times, contacts);
}

MonopedGaitGenerator::GaitInfo
MonopedGaitGenerator::GetStrideHopLong () const
{
  auto times =
  {
      0.2, 0.3,
  };
  auto contacts =
  {
      o_, x_,
  };

  return std::make_pair(times, contacts);
}

} /* namespace towr */
