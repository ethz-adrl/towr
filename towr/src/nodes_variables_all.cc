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

#include <towr/variables/nodes_variables_all.h>

namespace towr {

NodesVariablesAll::NodesVariablesAll (int n_nodes, int n_dim, std::string variable_id)
    : NodesVariables(variable_id)
{
  int n_opt_variables = n_nodes*Node::n_derivatives*n_dim;

  n_dim_ = n_dim;
  nodes_  = std::vector<Node>(n_nodes, Node(n_dim));
  bounds_ = VecBound(n_opt_variables, ifopt::NoBound);
  SetRows(n_opt_variables);
}

std::vector<NodesVariablesAll::NodeValueInfo>
NodesVariablesAll::GetNodeValuesInfo (int idx) const
{
  std::vector<NodeValueInfo> vec_nvi;

  int n_opt_values_per_node_ = 2*GetDim();
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  NodeValueInfo nvi;
  nvi.deriv_ = internal_id<GetDim()? kPos : kVel;
  nvi.dim_   = internal_id%GetDim();
  nvi.id_    = std::floor(idx/n_opt_values_per_node_);

  vec_nvi.push_back(nvi);

  return vec_nvi;
}

} /* namespace towr */
