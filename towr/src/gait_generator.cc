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

#include <towr/initialization/gait_generator.h>

#include <cassert>
#include <numeric>   // std::accumulate
#include <algorithm> // std::transform

#include <towr/initialization/monoped_gait_generator.h>
#include <towr/initialization/biped_gait_generator.h>
#include <towr/initialization/quadruped_gait_generator.h>

namespace towr {


GaitGenerator::Ptr
GaitGenerator::MakeGaitGenerator(int leg_count)
{
  switch (leg_count) {
    case 1: return std::make_shared<MonopedGaitGenerator>();   break;
    case 2: return std::make_shared<BipedGaitGenerator>();     break;
    case 4: return std::make_shared<QuadrupedGaitGenerator>(); break;
    default: assert(false); break; // Error: Not implemented
  }
}

GaitGenerator::VecTimes
GaitGenerator::GetPhaseDurations (double t_total, EE ee) const
{
  // scale total time tu t_total
  std::vector<double> durations;
  for (auto d : GetNormalizedPhaseDurations(ee))
    durations.push_back(d*t_total);

  return durations;
}

GaitGenerator::VecTimes
GaitGenerator::GetNormalizedPhaseDurations (EE ee) const
{
  auto v = GetPhaseDurations().at(ee); // shorthand
  double total_time = std::accumulate(v.begin(), v.end(), 0.0);
  std::transform(v.begin(), v.end(), v.begin(),
                 [total_time](double t_phase){ return t_phase/total_time;});

  return v;
}

GaitGenerator::FootDurations
GaitGenerator::GetPhaseDurations () const
{
  int n_ee = contacts_.front().size();
  VecTimes d_accumulated(n_ee, 0.0);

  FootDurations foot_durations(n_ee);
  for (int phase=0; phase<contacts_.size()-1; ++phase) {
    ContactState curr = contacts_.at(phase);
    ContactState next = contacts_.at(phase+1);

    for (int ee=0; ee<curr.size(); ++ee) {
      d_accumulated.at(ee) += times_.at(phase);

      // if contact will change in next phase, so this phase duration complete
      bool contacts_will_change = curr.at(ee) != next.at(ee);
      if (contacts_will_change)  {
        foot_durations.at(ee).push_back(d_accumulated.at(ee));
        d_accumulated.at(ee) = 0.0;
      }
    }
  }

  // push back last phase
  for (int ee=0; ee<contacts_.back().size(); ++ee)
    foot_durations.at(ee).push_back(d_accumulated.at(ee) + times_.back());


  return foot_durations;
}

bool
GaitGenerator::IsInContactAtStart (EE ee) const
{
  return contacts_.front().at(ee);
}

void
GaitGenerator::SetGaits (const std::vector<Gaits>& gaits)
{
  contacts_.clear();
  times_.clear();

  for (Gaits g : gaits) {
    auto info = GetGait(g);

    std::vector<double>       t = info.first;
    std::vector<ContactState> c = info.second;
    assert(t.size() == c.size()); // make sure every phase has a time

    times_.insert      (times_.end(), t.begin(), t.end());
    contacts_.insert(contacts_.end(), c.begin(), c.end());
  }
}

GaitGenerator::GaitInfo
GaitGenerator::RemoveTransition (const GaitInfo& g) const
{
  GaitInfo new_gait = g;

  // remove the final transition between strides
  // but ensure that last step duration is not cut off
  new_gait.first.pop_back();
  new_gait.first.back() += g.first.back();

  new_gait.second.pop_back();

  return new_gait;
}

} /* namespace towr */


