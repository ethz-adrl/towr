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

#include <towr/constraints/total_duration_constraint.h>
#include <towr/variables/variable_names.h>

namespace towr {


TotalDurationConstraint::TotalDurationConstraint (double T_total, int ee)
    :ConstraintSet(1, "totalduration-" + std::to_string(ee))
{
  T_total_ = T_total;
  ee_ = ee;
}

void
TotalDurationConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  phase_durations_ = x->GetComponent<PhaseDurations>(id::EESchedule(ee_));
}

Eigen::VectorXd
TotalDurationConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());
  g(0) = phase_durations_->GetValues().sum(); // attention: excludes last duration
  return g;
}

TotalDurationConstraint::VecBound
TotalDurationConstraint::GetBounds () const
{
  // TODO hacky and should be fixed
  // since last phase is not optimized over these hardcoded numbers go here
  double min_duration_last_phase = 0.2;
  return VecBound(GetRows(), ifopt::Bounds(0.1, T_total_-min_duration_last_phase));
}

void
TotalDurationConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == phase_durations_->GetName())
    for (int col=0; col<phase_durations_->GetRows(); ++col)
      jac.coeffRef(0, col) = 1.0;
}

} // namespace towr
