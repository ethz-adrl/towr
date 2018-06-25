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

#include <towr/variables/phase_durations.h>

#include <numeric> // std::accumulate

#include <towr/variables/variable_names.h>
#include <towr/variables/spline.h> // for Spline::GetSegmentID()


namespace towr {


PhaseDurations::PhaseDurations (EndeffectorID ee,
                                const VecDurations& timings,
                                bool is_first_phase_in_contact,
                                double min_duration,
                                double max_duration)
    // -1 since last phase-duration is not optimized over, but comes from total time
    :VariableSet(timings.size()-1, id::EESchedule(ee))
{
  durations_ = timings;
  t_total_ = std::accumulate(timings.begin(), timings.end(), 0.0);
  phase_duration_bounds_ = ifopt::Bounds(min_duration, max_duration);
  initial_contact_state_ = is_first_phase_in_contact;
}

void
PhaseDurations::AddObserver (PhaseDurationsObserver* const o)
{
  observers_.push_back(o);
}

void
PhaseDurations::UpdateObservers () const
{
  for (auto& spline : observers_)
    spline->UpdatePolynomialDurations();
}

Eigen::VectorXd
PhaseDurations::GetValues () const
{
  VectorXd x(GetRows());

  for (int i=0; i<x.rows(); ++i)
    x(i) = durations_.at(i);

  return x;
}

void
PhaseDurations::SetVariables (const VectorXd& x)
{
  for (int i=0; i<GetRows(); ++i)
    durations_.at(i) = x(i);

  durations_.back() =  t_total_ - x.sum();
  assert(durations_.back()>0);
  UpdateObservers();
}

PhaseDurations::VecBound
PhaseDurations::GetBounds () const
{
  VecBound bounds;

  for (int i=0; i<GetRows(); ++i)
    bounds.push_back(phase_duration_bounds_);

  return bounds;
}

PhaseDurations::VecDurations
PhaseDurations::GetPhaseDurations() const
{
  return durations_;
}

bool
PhaseDurations::IsContactPhase (double t) const
{
  int phase_id = Spline::GetSegmentID(t, durations_);
  return phase_id%2 == 0? initial_contact_state_ : !initial_contact_state_;
}

PhaseDurations::Jacobian
PhaseDurations::GetJacobianOfPos (int current_phase,
                                  const VectorXd& dx_dT,
                                  const VectorXd& xd) const
{
  int n_dim = xd.rows();
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(n_dim, GetRows());

  bool in_last_phase = (current_phase == durations_.size()-1);

  // duration of current phase expands and compressed spline
  if (!in_last_phase)
    jac.col(current_phase) = dx_dT;

  for (int phase=0; phase<current_phase; ++phase) {
    // each previous durations shifts spline along time axis
    jac.col(phase) = -1*xd;

    // in last phase previous duration cause expansion/compression of spline
    // as final time is fixed.
    if (in_last_phase)
      jac.col(phase) -= dx_dT;
  }

  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  // as durations change and t_global falls into different spline
  return jac.sparseView(1.0, -1.0);
}

} /* namespace towr */


