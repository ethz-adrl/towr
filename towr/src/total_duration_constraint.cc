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

#include <towr/constraints/total_duration_constraint.h>

#include <towr/variables/variable_names.h>

namespace towr {


TotalDurationConstraint::TotalDurationConstraint (double T_total, int ee)
    :ConstraintSet(1, "DurationConstraint_ee_-" + std::to_string(ee))
{
  T_total_ = T_total;
  ee_ = ee;
}

void
TotalDurationConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  schedule_ = std::dynamic_pointer_cast<ContactSchedule>(x->GetComponent(id::EESchedule(ee_)));
}

Eigen::VectorXd
TotalDurationConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());
//  for (double t_phase : schedule_->GetTimePerPhase())
//    g(0) += t_phase;
//
  g(0) = schedule_->GetValues().sum();
  return g;
}

TotalDurationConstraint::VecBound
TotalDurationConstraint::GetBounds () const
{
  return VecBound(GetRows(), ifopt::Bounds(0.1, T_total_-0.2));
}

void
TotalDurationConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == schedule_->GetName())
    for (int col=0; col<schedule_->GetRows(); ++col)
      jac.coeffRef(0, col) = 1.0;
}


} // namespace towr
