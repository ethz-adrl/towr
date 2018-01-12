/**
 @file    node_values.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#include <towr/variables/node_values.h>

#include <array>
#include <cmath>
#include <numeric>
#include <tuple>
#include <utility>
#include <Eigen/Eigen>

namespace towr {

using namespace ifopt;
using namespace xpp;


NodeValues::NodeValues (int n_dim, const std::string& name)
    : VariableSet(kSpecifyLater, name)
{
  n_dim_ = n_dim;
}

NodeValues::NodeValues (int n_dim, int n_nodes, const std::string& name)
    : NodeValues(n_dim, name)
{
  int n_variables = n_nodes*2*n_dim;
  InitMembers(n_nodes, n_variables);
}

void
NodeValues::InitMembers(int n_nodes, int n_variables)
{
  nodes_  = std::vector<Node>(n_nodes);
  bounds_ = VecBound(n_variables, NoBound);
  SetRows(n_variables);
}

void
NodeValues::InitializeNodes(const VectorXd& initial_pos,
                            const VectorXd& final_pos,
                            double t_total)
{
  VectorXd dp = final_pos-initial_pos;
  VectorXd average_velocity = dp/t_total;
  int num_nodes = nodes_.size();
  for (int i=0; i<nodes_.size(); ++i) {
    Node n;
    n.val_ = initial_pos + i/static_cast<double>(num_nodes-1)*dp;
    n.deriv_ = average_velocity;
    nodes_.at(i) = n;
  }
}

int
NodeValues::Index(int node_id, Deriv deriv, int dim) const
{
  IndexInfo n;
  n.node_id_ = node_id;
  n.node_deriv_   = deriv;
  n.node_deriv_dim_     = dim;

  // could also cache this as map for more efficiency, but adding complexity
  for (int idx=0; idx<GetRows(); ++idx)
    for ( IndexInfo node_info : GetNodeInfoAtOptIndex(idx))
      if ( node_info == n )
        return idx;

  assert(false); // index representing these quantities doesn't exist
}

// reverse of the above
std::vector<NodeValues::IndexInfo>
NodeValues::GetNodeInfoAtOptIndex (int idx) const
{
  std::vector<IndexInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*n_dim_;
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  IndexInfo node;
  node.node_deriv_   = internal_id<n_dim_? kPos : kVel;
  node.node_deriv_dim_     = internal_id%n_dim_;
  node.node_id_ = std::floor(idx/n_opt_values_per_node_);

  nodes.push_back(node);

  return nodes;
}

VectorXd
NodeValues::GetValues () const
{
  VectorXd x(GetRows());

  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      x(idx) = nodes_.at(info.node_id_).at(info.node_deriv_)(info.node_deriv_dim_);

  return x;
}

void
NodeValues::SetVariables (const VectorXd& x)
{
  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      nodes_.at(info.node_id_).at(info.node_deriv_)(info.node_deriv_dim_) = x(idx);

  UpdateObservers();
}

void
NodeValues::UpdateObservers() const
{
  for (auto& o : observers_)
    o->UpdatePolynomials();
}

void
NodeValues::AddObserver(NodesObserver* const o)
{
   observers_.push_back(o);
}

NodeValues::VecDurations
NodeValues::ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const
{
  return phase_durations; // default is do nothing
}

double
NodeValues::GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const
{
  return 1.0; // default every polynomial represents one phase
}

int
NodeValues::GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const
{
  return 0; // every phase is represented by single polynomial
}

int
NodeValues::GetNodeId (int poly_id, Side side)
{
  return poly_id + side;
}

const std::vector<NodeValues::Node>
NodeValues::GetBoundaryNodes(int poly_id) const
{
  std::vector<Node> nodes;
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::Start)));
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::End)));
  return nodes;
}

int
NodeValues::GetDim() const
{
  return n_dim_;
}

int
NodeValues::GetPolynomialCount() const
{
  return nodes_.size() - 1;
}

NodeValues::VecBound
NodeValues::GetBounds () const
{
  return bounds_;
}

const std::vector<NodeValues::Node>
NodeValues::GetNodes() const
{
  return nodes_;
}

void
NodeValues::AddBounds(int node_id, Deriv deriv,
                      const std::vector<int>& dimensions,
                      const VectorXd& val)
{
  for (auto dim : dimensions)
    AddBound(node_id, deriv, dim, val(dim));
}

void
NodeValues::AddBound (int node_id, Deriv d, int dim, double val)
{
  for (int idx=0; idx<GetRows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      if (info.node_id_==node_id && info.node_deriv_==d && info.node_deriv_dim_==dim)
        bounds_.at(idx) = Bounds(val, val);
}

void
NodeValues::AddStartBound (Deriv d,
                           const std::vector<int>& dimensions,
                           const VectorXd& val)
{
  AddBounds(0, d, dimensions, val);
}

void
NodeValues::AddFinalBound (Deriv deriv,
                           const std::vector<int>& dimensions,
                           const VectorXd& val)
{
  AddBounds(nodes_.size()-1, deriv, dimensions, val);
}

int
NodeValues::IndexInfo::operator==(const IndexInfo& right) const
{
  return (node_id_ == right.node_id_)
      && (node_deriv_   == right.node_deriv_)
      && (node_deriv_dim_     == right.node_deriv_dim_);
};

} /* namespace towr */
