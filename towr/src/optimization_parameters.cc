/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/optimization_parameters.h>

#include <algorithm>

namespace towr {

OptimizationParameters::OptimizationParameters ()
{
  // dynamic constraints are enforced at this interval as well
  dt_base_polynomial_ = 0.2; // 0.2


  // 2 also works quite well. Remember that inbetween the nodes, forces
  // could still be violating unilateral and friction constraints by
  // polynomial interpolation
  force_splines_per_stance_phase_ = 3; // this makes it a lot bigger


  // range of motion constraint
  dt_range_of_motion_ = 0.1; // 0.1
  ee_splines_per_swing_phase_ = 2; // should always be 2 if i want to use swing constraint!

  dt_base_range_of_motion_ = dt_base_polynomial_/4.;


  min_phase_duration_ = 0.1;
  double max_time = 2.0; // this helps convergence
  max_phase_duration_ = max_time>GetTotalTime()?  GetTotalTime() : max_time;
//  max_phase_duration_ = GetTotalTime()/contact_timings_.size();


  force_z_limit_ = 10000; // N


  constraints_ = {
      EndeffectorRom,
      Dynamic,
      Terrain,
      Force,
//      TotalTime, // Attention: this causes segfault in SNOPT
      Swing, // this is important for lifting leg
//      BaseRom, //  CAREFUL: restricts the base to be in a specific range->very limiting
  };

  cost_weights_ = {
//      {ForcesCostID, 1.0},
//      {ComCostID, 1.0}
  };
}

OptimizationParameters::CostWeights
OptimizationParameters::GetCostWeights () const
{
  return cost_weights_;
}

OptimizationParameters::UsedConstraints
OptimizationParameters::GetUsedConstraints () const
{
  return constraints_;
}

void
OptimizationParameters::SetPhaseDurations (
    const std::vector<VecTimes>& phase_durations,
    const std::vector<bool>& initial_contact)
{
  ee_phase_durations_ = phase_durations;
  ee_in_contact_at_start_ = initial_contact;
}

bool
OptimizationParameters::OptimizeTimings () const
{
  ConstraintName c = TotalTime;
  auto v = constraints_; // shorthand
  return std::find(v.begin(), v.end(), c) != v.end();
}

OptimizationParameters::VecTimes
OptimizationParameters::GetBasePolyDurations () const
{
  std::vector<double> base_spline_timings_;
  double dt = dt_base_polynomial_;
  double t_left = t_total_;

  double eps = 1e-10; // since repeated subtraction causes inaccuracies
  while (t_left > eps) {
    double duration = t_left>dt?  dt : t_left;
    base_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return base_spline_timings_;
}

OptimizationParameters::VecTimes
OptimizationParameters::GetEEPhaseDurations(EEID ee) const
{
  return ee_phase_durations_.at(ee);
}

int
OptimizationParameters::GetPhaseCount(EEID ee) const
{
  return ee_phase_durations_.at(ee).size();
}

bool
OptimizationParameters::IsEEInContactAtStart(EEID ee) const
{
  return ee_in_contact_at_start_.at(ee);
}

} // namespace towr

