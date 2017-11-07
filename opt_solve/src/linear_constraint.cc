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

#include <opt_solve/linear_constraint.h>

namespace opt {


LinearEqualityConstraint::LinearEqualityConstraint (
  const VariablesPtr& variables,
  const Eigen::MatrixXd& M,
  const Eigen::VectorXd& v,
  const std::string& variable_name)
    : Constraint(variables, v.rows(), "LinearEqualityConstraint-" + variable_name)
{
  M_ = M;
  v_ = v;
  variable_name_   = variable_name;
}

LinearEqualityConstraint::VectorXd
LinearEqualityConstraint::GetValues () const
{
  VectorXd x = GetVariables()->GetComponent(variable_name_)->GetValues();
  return M_*x;
}

LinearEqualityConstraint::VecBound
LinearEqualityConstraint::GetBounds () const
{
  VecBound bounds;

  for (int i=0; i<GetRows(); ++i) {
    Bounds bound(-v_[i],-v_[i]);
    bounds.push_back(bound);
  }

  return bounds;
}

void
LinearEqualityConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  // the constraints are all linear w.r.t. the decision variables.
  // careful, sparseView is only valid when the Jacobian is constant
  if (var_set == variable_name_)
    jac = M_.sparseView();
}

LinearEqualityConstraint::~LinearEqualityConstraint ()
{
}


} /* namespace opt */

