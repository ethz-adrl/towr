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

/**
 @file    cost_adapter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2016
 @brief   Defines the class SoftConstraint
 */

#include <opt_solve/soft_constraint.h>


namespace opt {

SoftConstraint::SoftConstraint (const ConstraintPtr& constraint, double weight)
    :Component(1, "SoftConstraint-" + constraint->GetName())
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
  weights_.resize(n_constraints);
  weights_.setOnes();

  weight_ = weight;
}

SoftConstraint::~SoftConstraint ()
{
}

SoftConstraint::VectorXd
SoftConstraint::GetValues () const
{
  VectorXd g = constraint_->GetValues();
  VectorXd cost = 0.5*(g-b_).transpose()*weights_.asDiagonal()*(g-b_);
  return weight_ * cost;
}

SoftConstraint::Jacobian
SoftConstraint::GetJacobian () const
{
  VectorXd g   = constraint_->GetValues();
  Jacobian jac = constraint_->GetJacobian();
  VectorXd grad = weight_*jac.transpose()*weights_.asDiagonal()*(g-b_);
  return grad.transpose().sparseView();
}

} /* namespace opt */
