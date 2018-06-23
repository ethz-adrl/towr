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

#include <towr/costs/soft_constraint.h>

namespace towr {

SoftConstraint::SoftConstraint (const ConstraintPtr& constraint)
    :Component(1, "soft-" + constraint->GetName())
{
  constraint_ = constraint;
  int n_constraints = constraint_->GetRows();

  // average value of each upper and lower bound
  b_ = VectorXd(n_constraints);
  int i=0;
  for (auto b : constraint_->GetBounds()) {
    b_(i++) = (b.upper_ + b.lower_)/2.;
  }

  // treat all constraints equally by default
  W_.resize(n_constraints);
  W_.setOnes();
}

SoftConstraint::VectorXd
SoftConstraint::GetValues () const
{
  VectorXd g = constraint_->GetValues();
  VectorXd cost = 0.5*(g-b_).transpose()*W_.asDiagonal()*(g-b_);
  return cost;
}

SoftConstraint::Jacobian
SoftConstraint::GetJacobian () const
{
  VectorXd g   = constraint_->GetValues();
  Jacobian jac = constraint_->GetJacobian();
  VectorXd grad = jac.transpose()*W_.asDiagonal()*(g-b_);
  return grad.transpose().sparseView();
}

} /* namespace towr */
