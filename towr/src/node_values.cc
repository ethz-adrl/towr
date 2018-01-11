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

NodeValues::NodeValues (int n_dim, int n_polynomials, const std::string& name)
    : NodeValues(n_dim, BuildPolyInfos(n_polynomials), name)
{
}


NodeValues::NodeValues (int n_dim, const PolyInfoVec& poly_infos, const std::string& name)
    : VariableSet(kSpecifyLater, name)
{
  n_dim_ = n_dim;

  polynomial_info_ = poly_infos;

//  SetNodeMappings();
  nodes_  = std::vector<Node>(poly_infos.size()+1);
  int n_opt_variables = nodes_.size() * 2*n_dim_;
  SetRows(n_opt_variables);
  bounds_ = VecBound(GetRows(), NoBound);

  // default, non initialized values
  CacheNodeInfoToIndexMappings();
}


NodeValues::PolyInfoVec
NodeValues::BuildPolyInfos(int num_polys) const
{
  PolyInfoVec poly_infos;

  for (int i=0; i<num_polys; ++i) {
    PolyInfo info;
    info.is_constant_ = false; // always use different node for start and end
    info.num_polys_in_phase_ = 1;
    info.phase_ = i;
    info.poly_id_in_phase_ = 0;

    poly_infos.push_back(info);
  }

  return poly_infos;
}


void
NodeValues::InitializeVariables(const VectorXd& initial_pos,
                                const VectorXd& final_pos,
                                double t_total)
{
  VectorXd dp = final_pos-initial_pos;
  VectorXd average_velocity = dp/t_total;
  int num_nodes = nodes_.size();
  for (int i=0; i<nodes_.size(); ++i) {
    Node n;
    n.at(kPos) = initial_pos + i/static_cast<double>(num_nodes-1)*dp;
    n.at(kVel) = average_velocity;
    nodes_.at(i) = n;
  }
}


//void
//NodeValues::SetNodeMappings ()
//{
//  int opt_id = 0;
//  for (int i=0; i<polynomial_info_.size(); ++i) {
//    int node_id_start = GetNodeId(i, CubicHermitePoly::Start);
//
//    optnode_to_node_[opt_id].push_back(node_id_start);
//    // use same value for next node if polynomial is constant
//    if (!polynomial_info_.at(i).is_constant_)
//      opt_id++;
//  }
//
//  int last_node_id = polynomial_info_.size();
//  optnode_to_node_[opt_id].push_back(last_node_id);
//}

void
NodeValues::CacheNodeInfoToIndexMappings ()
{
  for (int idx=0; idx<GetRows(); ++idx) {
    for (auto info : GetNodeInfoAtOptIndex(idx)) {
      node_info_to_idx[info] = idx;
    }
  }
}

// reverse of the below
// remember that not every node is optimized over, but some are put together
int
NodeValues::Index(int node_id, MotionDerivative deriv, int dim) const
{
  NodeInfo n;
  n.id_ = node_id;
  n.deriv_ = deriv;
  n.dim_ = dim;
  return node_info_to_idx.at(n);
}

// reverse of the above
std::vector<NodeValues::NodeInfo>
NodeValues::GetNodeInfoAtOptIndex (int idx) const
{
  std::vector<NodeInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*n_dim_;
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  NodeInfo node;
  node.deriv_ = internal_id<n_dim_? kPos : kVel;
//  node.deriv_ = static_cast<MotionDerivative>(std::floor(internal_id/n_dim_));
  node.dim_   = internal_id%n_dim_;
//  node.dim_   = internal_id-node.deriv_*n_dim_;

  node.id_ = std::floor(idx/n_opt_values_per_node_);
  nodes.push_back(node);

//  int opt_node = std::floor(idx/n_opt_values_per_node_);
//  for (auto node_id : optnode_to_node_.at(opt_node)) {
//    node.id_ = node_id;
//    nodes.push_back(node);
//  }

  return nodes;
}

VectorXd
NodeValues::GetValues () const
{
  VectorXd x(GetRows());

  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      x(idx) = nodes_.at(info.id_).at(info.deriv_)(info.dim_);

  return x;
}

void
NodeValues::SetVariables (const VectorXd& x)
{
  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      nodes_.at(info.id_).at(info.deriv_)(info.dim_) = x(idx);

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

void
NodeValues::AddBounds(int node_id, MotionDerivative deriv,
                      const std::vector<int>& dimensions,
                      const VectorXd& val)
{
  for (auto dim : dimensions)
    AddBound(node_id, deriv, dim, val(dim));
}

void
NodeValues::AddBound (int node_id, MotionDerivative d, int dim, double val)
{
  for (int idx=0; idx<GetRows(); ++idx)
    for (auto info : GetNodeInfoAtOptIndex(idx))
      if (info.id_==node_id && info.deriv_==d && info.dim_==dim)
        bounds_.at(idx) = Bounds(val, val);
}

void
NodeValues::AddStartBound (MotionDerivative d,
                           const std::vector<int>& dimensions,
                           const VectorXd& val)
{
  AddBounds(0, d, dimensions, val);
}

void
NodeValues::AddFinalBound (MotionDerivative deriv,
                           const std::vector<int>& dimensions,
                           const VectorXd& val)
{
//  // careful to not overwrite start bound
//  if (nodes_.size() == 2)
//    throw std::runtime_error("overwriting start bound");

  AddBounds(nodes_.size()-1, deriv, dimensions, val);
}

int
NodeValues::GetNodeId (int poly_id, Side side) const
{
  return poly_id + side;
}

// returns the two nodes that make up polynomial with "poly_id"
const std::vector<NodeValues::Node>
NodeValues::GetBoundaryNodes(int poly_id) const
{
  std::vector<Node> nodes;
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::Start)));
  nodes.push_back(nodes_.at(GetNodeId(poly_id, Side::End)));
  return nodes;
};

std::vector<int>
NodeValues::GetAdjacentPolyIds (int node_id) const
{
  std::vector<int> poly_ids;
  int last_node_id = nodes_.size()-1;

  if (node_id==0)
    poly_ids.push_back(0);
  else if (node_id==last_node_id)
    poly_ids.push_back(last_node_id-1);
  else {
    poly_ids.push_back(node_id-1);
    poly_ids.push_back(node_id);
  }

  return poly_ids;
}

//void
//NodeValues::SetBoundsAboveGround ()
//{
//  double z_height = 0.0;
//  for (int idx=0; idx<GetRows(); ++idx)
//    for (auto info : GetNodeInfo(idx))
//      if (info.deriv_==kPos && info.dim_==Z)
//        bounds_.at(idx) = Bounds(z_height, +1.0e20);
//}

} /* namespace towr */
