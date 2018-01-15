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

#include <towr/costs/node_cost.h>

#include <cmath>
#include <Eigen/Eigen>

#include <towr/variables/cartesian_dimensions.h>


namespace towr {

NodeCost::NodeCost (const std::string& nodes_id) : CostTerm("Node Cost")
{
  node_id_ = nodes_id;

  deriv_ = kPos;
  dim_   = Z;
}

void
NodeCost::InitVariableDependedQuantities (const VariablesPtr& x)
{
  nodes_ = std::dynamic_pointer_cast<NodeVariables>(x->GetComponent(node_id_));
}

double
NodeCost::GetCost () const
{
  double cost;
  for (auto n : nodes_->GetNodes()) {
    double val = n.at(deriv_)(dim_);
    cost += std::pow(val,2);
  }

  return cost;
}

void
NodeCost::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == node_id_) {

    for (int idx=0; idx<nodes_->GetRows(); ++idx)
      for (auto n : nodes_->GetNodeInfoAtOptIndex(idx))
        if (n.node_deriv_==deriv_ && n.node_deriv_dim_==dim_) {
          double val = nodes_->GetNodes().at(n.node_id_).at(deriv_)(dim_);
          jac.coeffRef(0, idx) += 2.0*val;
        }
  }
}

} /* namespace towr */


