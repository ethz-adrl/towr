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

#include <towr/variables/nodes_variables.h>

namespace towr {


NodesVariables::NodesVariables (const std::string& name)
    : VariableSet(kSpecifyLater, name)
{
}

int
NodesVariables::GetOptIndex(const NodeValueInfo& nvi_des) const
{
  // could also cache this as map for more efficiency, but adding complexity
  for (int idx=0; idx<GetRows(); ++idx)
    for ( NodeValueInfo nvi : GetNodeValuesInfo(idx))
      if ( nvi == nvi_des )
        return idx;

  return NodeValueNotOptimized; // index representing these quantities doesn't exist
}

Eigen::VectorXd
NodesVariables::GetValues () const
{
  VectorXd x(GetRows());

  for (int idx=0; idx<x.rows(); ++idx)
    for (auto nvi : GetNodeValuesInfo(idx))
      x(idx) = nodes_.at(nvi.id_).at(nvi.deriv_)(nvi.dim_);

  return x;
}

void
NodesVariables::SetVariables (const VectorXd& x)
{
  for (int idx=0; idx<x.rows(); ++idx)
    for (auto nvi : GetNodeValuesInfo(idx))
      nodes_.at(nvi.id_).at(nvi.deriv_)(nvi.dim_) = x(idx);

  UpdateObservers();
}

void
NodesVariables::UpdateObservers() const
{
  for (auto& o : observers_)
    o->UpdateNodes();
}

void
NodesVariables::AddObserver(ObserverPtr const o)
{
   observers_.push_back(o);
}

int
NodesVariables::GetNodeId (int poly_id, Side side)
{
  return poly_id + side;
}

const std::vector<Node>
NodesVariables::GetBoundaryNodes(int poly_id) const
{
  std::vector<Node> nodes;
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::Start)));
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::End)));
  return nodes;
}

int
NodesVariables::GetDim() const
{
  return n_dim_;
}

int
NodesVariables::GetPolynomialCount() const
{
  return nodes_.size() - 1;
}

NodesVariables::VecBound
NodesVariables::GetBounds () const
{
  return bounds_;
}

const std::vector<Node>
NodesVariables::GetNodes() const
{
  return nodes_;
}

void
NodesVariables::SetByLinearInterpolation(const VectorXd& initial_val,
                                         const VectorXd& final_val,
                                         double t_total)
{
  // only set those that are part of optimization variables,
  // do not overwrite phase-based parameterization
  VectorXd dp = final_val-initial_val;
  VectorXd average_velocity = dp / t_total;
  int num_nodes = nodes_.size();

  for (int idx=0; idx<GetRows(); ++idx) {
    for (auto nvi : GetNodeValuesInfo(idx)) {

      if (nvi.deriv_ == kPos) {
        VectorXd pos = initial_val + nvi.id_/static_cast<double>(num_nodes-1)*dp;
        nodes_.at(nvi.id_).at(kPos)(nvi.dim_) = pos(nvi.dim_);
      }

      if (nvi.deriv_ == kVel) {
        nodes_.at(nvi.id_).at(kVel)(nvi.dim_) = average_velocity(nvi.dim_);
      }
    }
  }
}

void
NodesVariables::AddBounds(int node_id, Dx deriv,
                 const std::vector<int>& dimensions,
                 const VectorXd& val)
{
  for (auto dim : dimensions)
    AddBound(NodeValueInfo(node_id, deriv, dim), val(dim));
}

void
NodesVariables::AddBound (const NodeValueInfo& nvi_des, double val)
{
  for (int idx=0; idx<GetRows(); ++idx)
    for (auto nvi : GetNodeValuesInfo(idx))
      if (nvi == nvi_des)
        bounds_.at(idx) = ifopt::Bounds(val, val);
}

void
NodesVariables::AddStartBound (Dx d, const std::vector<int>& dimensions, const VectorXd& val)
{
  AddBounds(0, d, dimensions, val);
}

void
NodesVariables::AddFinalBound (Dx deriv, const std::vector<int>& dimensions,
                      const VectorXd& val)
{
  AddBounds(nodes_.size()-1, deriv, dimensions, val);
}

NodesVariables::NodeValueInfo::NodeValueInfo(int node_id, Dx deriv, int node_dim)
{
  id_    = node_id;
  deriv_ = deriv;
  dim_   = node_dim;
}

int
NodesVariables::NodeValueInfo::operator==(const NodeValueInfo& right) const
{
  return (id_    == right.id_)
      && (deriv_ == right.deriv_)
      && (dim_   == right.dim_);
};

} /* namespace towr */
