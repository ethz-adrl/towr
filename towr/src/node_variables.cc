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

#include <towr/variables/node_variables.h>


namespace towr {


NodeVariables::NodeVariables (int n_dim, const std::string& name)
    : VariableSet(kSpecifyLater, name)
{
  n_dim_ = n_dim;
}

void
NodeVariables::InitMembers(int n_nodes, int n_variables)
{
  nodes_  = std::vector<Node>(n_nodes, Node(n_dim_));
  bounds_ = VecBound(n_variables, ifopt::NoBound);
  SetRows(n_variables);
}

void
NodeVariables::InitializeNodesTowardsGoal(const VectorXd& initial_pos,
                               const VectorXd& final_pos,
                               double t_total)
{
  VectorXd dp = final_pos-initial_pos;
  VectorXd average_velocity = dp/t_total;
  int num_nodes = nodes_.size();
  for (int i=0; i<nodes_.size(); ++i) {
    Node n(n_dim_);
    n.at(kPos) = initial_pos + i/static_cast<double>(num_nodes-1)*dp;
    n.at(kVel) = average_velocity;
    nodes_.at(i) = n;
  }
}

NodeVariables::IndexInfo::IndexInfo(int node_id, Dx deriv, int node_dim)
{
  node_id_    = node_id;
  node_deriv_ = deriv;
  node_dim_   = node_dim;
}

int
NodeVariables::Index(int node_id, Dx deriv, int node_dim) const
{
  return Index(IndexInfo(node_id, deriv, node_dim));
}

int
NodeVariables::Index(const IndexInfo& n) const
{
  // could also cache this as map for more efficiency, but adding complexity
  for (int idx=0; idx<GetRows(); ++idx)
    for ( IndexInfo node_info : GetNodeInfoAtOptIndex(idx))
      if ( node_info == n )
        return idx;

  assert(false); // index representing these quantities doesn't exist
}

// reverse of the above
std::vector<NodeVariables::IndexInfo>
NodeVariables::GetNodeInfoAtOptIndex (int idx) const
{
  std::vector<IndexInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*n_dim_;
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  IndexInfo node;
  node.node_deriv_ = internal_id<n_dim_? kPos : kVel;
  node.node_dim_   = internal_id%n_dim_;
  node.node_id_    = std::floor(idx/n_opt_values_per_node_);

  nodes.push_back(node);

  return nodes;
}

Eigen::VectorXd
NodeVariables::GetValues () const
{
  VectorXd x(GetRows());

  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      x(idx) = nodes_.at(info.node_id_).at(info.node_deriv_)(info.node_dim_);

  return x;
}

void
NodeVariables::SetVariables (const VectorXd& x)
{
  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      nodes_.at(info.node_id_).at(info.node_deriv_)(info.node_dim_) = x(idx);

  UpdateObservers();
}

void
NodeVariables::UpdateObservers() const
{
  for (auto& o : observers_)
    o->UpdatePolynomials();
}

void
NodeVariables::AddObserver(NodesObserver* const o)
{
   observers_.push_back(o);
}

int
NodeVariables::GetNodeId (int poly_id, Side side)
{
  return poly_id + side;
}

const std::vector<Node>
NodeVariables::GetBoundaryNodes(int poly_id) const
{
  std::vector<Node> nodes;
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::Start)));
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::End)));
  return nodes;
}

int
NodeVariables::GetDim() const
{
  return n_dim_;
}

int
NodeVariables::GetPolynomialCount() const
{
  return nodes_.size() - 1;
}

NodeVariables::VecBound
NodeVariables::GetBounds () const
{
  return bounds_;
}

const std::vector<Node>
NodeVariables::GetNodes() const
{
  return nodes_;
}

void
NodeVariables::AddBounds(int node_id, Dx deriv,
                         const std::vector<int>& dimensions,
                         const VectorXd& val)
{
  for (auto dim : dimensions)
    AddBound(IndexInfo(node_id, deriv, dim), val(dim));
}

void
NodeVariables::AddBound (const IndexInfo& node_info, double val)
{
  for (int idx=0; idx<GetRows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      if (info == node_info)
        bounds_.at(idx) = ifopt::Bounds(val, val);
}

void
NodeVariables::AddStartBound (Dx d,
                           const std::vector<int>& dimensions,
                           const VectorXd& val)
{
  AddBounds(0, d, dimensions, val);
}

void
NodeVariables::AddFinalBound (Dx deriv,
                           const std::vector<int>& dimensions,
                           const VectorXd& val)
{
  AddBounds(nodes_.size()-1, deriv, dimensions, val);
}

int
NodeVariables::IndexInfo::operator==(const IndexInfo& right) const
{
  return (node_id_    == right.node_id_)
      && (node_deriv_ == right.node_deriv_)
      && (node_dim_   == right.node_dim_);
};

} /* namespace towr */
