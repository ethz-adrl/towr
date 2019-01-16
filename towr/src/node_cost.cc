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

#include <towr/costs/node_cost.h>

#include <cmath>

namespace towr {

NodeCost::NodeCost (const std::string& nodes_id, Dx deriv, int dim, double weight)
    : CostTerm(nodes_id
               +"-dx_"+std::to_string(deriv)
               +"-dim_"+std::to_string(dim))
{
  node_id_ = nodes_id;
  deriv_ = deriv;
  dim_   = dim;
  weight_ = weight;
}

void
NodeCost::InitVariableDependedQuantities (const VariablesPtr& x)
{
  nodes_ = x->GetComponent<NodesVariables>(node_id_);
}

double
NodeCost::GetCost () const
{
  double cost;
  for (auto n : nodes_->GetNodes()) {
    double val = n.at(deriv_)(dim_);
    cost += weight_*std::pow(val,2);
  }

  return cost;
}

void
NodeCost::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == node_id_) {
    for (int i=0; i<nodes_->GetRows(); ++i)
      for (auto nvi : nodes_->GetNodeValuesInfo(i))
        if (nvi.deriv_==deriv_ && nvi.dim_==dim_) {
          double val = nodes_->GetNodes().at(nvi.id_).at(deriv_)(dim_);
          jac.coeffRef(0, i) += weight_*2.0*val;
        }
  }
}

} /* namespace towr */

