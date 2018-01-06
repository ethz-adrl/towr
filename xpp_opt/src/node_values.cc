/**
 @file    node_values.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#include <../include/xpp_opt/variables/node_values.h>

#include <array>
#include <cmath>
#include <numeric>
#include <tuple>
#include <utility>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace xpp {

using namespace opt;

NodeValues::NodeValues (int n_dim, int n_polynomials, const std::string& name)
    : NodeValues(n_dim, BuildPolyInfos(n_polynomials), name)
{
}

NodeValues::NodeValues (int n_dim, const PolyInfoVec& poly_infos, const std::string& name)
    : VariableSet(kSpecifyLater, name)
{
  n_dim_ = n_dim;

  polynomial_info_ = poly_infos;
  int n_polys = polynomial_info_.size();

  for (auto& infos : poly_infos) {
    auto p = std::make_shared<PolyType>(n_dim_);
    cubic_polys_.push_back(p);
  }

  SetNodeMappings();
  int n_opt_variables = opt_to_spline_.size() * 2*n_dim_;
  SetRows(n_opt_variables);

  // default, non initialized values
  nodes_  = std::vector<Node>(poly_infos.size()+1);
  poly_durations_ = VecDurations(poly_infos.size(), 0.0);

  bounds_ = VecBound(GetRows(), NoBound);
  SetIndexMappings();

  jac_structure_ = Jacobian(n_dim, n_opt_variables);
}

NodeValues::~NodeValues () {}


void
NodeValues::InitializeVariables(const VectorXd& initial_pos,
                                const VectorXd& final_pos,
                                const VecDurations& poly_durations)
{
  poly_durations_ = poly_durations;

  double t_total = std::accumulate(poly_durations.begin(), poly_durations.end(), 0.0);
  VectorXd dp = final_pos-initial_pos;
  VectorXd average_velocity = dp/t_total;
  int num_nodes = polynomial_info_.size()+1;
  for (int i=0; i<num_nodes; ++i) {
    Node n;
    n.at(kPos) = initial_pos + i/static_cast<double>(num_nodes-1)*dp;
    n.at(kVel) = average_velocity;
    nodes_.at(i) = n;
  }

  UpdatePolynomials();
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
NodeValues::SetNodeMappings ()
{
  int opt_id = 0;
  for (int i=0; i<polynomial_info_.size(); ++i) {
    int node_id_start = GetNodeId(i, CubicHermitePoly::Start);

    opt_to_spline_[opt_id].push_back(node_id_start);
    // use same value for next node if polynomial is constant
    if (!polynomial_info_.at(i).is_constant_)
      opt_id++;
  }

  int last_node_id = polynomial_info_.size();
  opt_to_spline_[opt_id].push_back(last_node_id);
}

void
NodeValues::SetIndexMappings ()
{
  for (int idx=0; idx<GetRows(); ++idx) {
    for (auto info : GetNodeInfo(idx)) {
      node_info_to_idx[info] = idx;
    }
  }
}

int
NodeValues::Index(int id, MotionDerivative deriv, int dim) const
{
  NodeInfo n;
  n.id_ = id;
  n.deriv_ = deriv;
  n.dim_ = dim;
  return node_info_to_idx.at(n);
}

std::vector<NodeValues::NodeInfo>
NodeValues::GetNodeInfo (int idx) const
{
  std::vector<NodeInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*n_dim_;
  int internal_id = idx%n_opt_values_per_node_; // 0...6

  NodeInfo node;
  node.deriv_ = static_cast<MotionDerivative>(std::floor(internal_id/n_dim_));
  node.dim_   = internal_id-node.deriv_*n_dim_;

  int opt_node = std::floor(idx/n_opt_values_per_node_);
  for (auto node_id : opt_to_spline_.at(opt_node)) {
    node.id_ = node_id;
    nodes.push_back(node);
  }

  return nodes;
}


VectorXd
NodeValues::GetValues () const
{
  VectorXd x(GetRows());

  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfo(idx))
      x(idx) = nodes_.at(info.id_).at(info.deriv_)(info.dim_);

  return x;
}

void
NodeValues::SetVariables (const VectorXd& x)
{
  for (int idx=0; idx<x.rows(); ++idx)
    for (auto info : GetNodeInfo(idx))
      nodes_.at(info.id_).at(info.deriv_)(info.dim_) = x(idx);

  UpdatePolynomials();
}


void
NodeValues::UpdatePolynomials ()
{
  for (int i=0; i<cubic_polys_.size(); ++i) {
    cubic_polys_.at(i)->SetNodes(nodes_.at(GetNodeId(i,Side::Start)),
                                 nodes_.at(GetNodeId(i,Side::End)),
                                 poly_durations_.at(i));
  }
}

const StateLinXd
NodeValues::GetPoint(double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);
  return cubic_polys_.at(id)->GetPoint(t_local);
}


NodeValues::Jacobian
NodeValues::GetJacobian (double t_global,  MotionDerivative dxdt) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);


  // if durations change, the polynomial active at a specified global time
  // changes. Therefore, all elements of the Jacobian could be non-zero
  if (fill_jacobian_structure_) {
    // assume every global time time can fall into every polynomial
    for (int i=0; i<polynomial_info_.size(); ++i)
      FillJacobian(i, 0.0, dxdt, jac_structure_, true);
  }
  fill_jacobian_structure_ = false;


  Jacobian jac(n_dim_, GetRows());
  if (durations_change_)
    jac = jac_structure_;

  FillJacobian(id, t_local, dxdt, jac);

  // needed to avoid Eigen::assert failure "wrong storage order" triggered
  // in dynamic_constraint.cc
  jac.makeCompressed();
  return jac;
}

void
NodeValues::FillJacobian (int poly_id, double t_local, MotionDerivative dxdt,
                          Jacobian& jac, bool fill_with_zeros) const
{
  for (int idx=0; idx<jac.cols(); ++idx) {
    for (NodeInfo info : GetNodeInfo(idx)) {
      for (Side side : {Side::Start, Side::End}) { // every jacobian is affected by two nodes

        int node = GetNodeId(poly_id,side);

        if (node == info.id_) {
          double val = cubic_polys_.at(poly_id)->GetDerivativeOf(dxdt, side, info.deriv_, t_local);

          // if only want structure
          if (fill_with_zeros)
            val = 0.0;

          jac.coeffRef(info.dim_, idx) += val;
        }
      }
    }
  }
}

void
NodeValues::SetBoundsAboveGround ()
{
  double z_height = 0.0;
  for (int idx=0; idx<GetRows(); ++idx)
    for (auto info : GetNodeInfo(idx))
      if (info.deriv_==kPos && info.dim_==Z)
        bounds_.at(idx) = Bounds(z_height, +1.0e20);
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
    for (auto info : GetNodeInfo(idx))
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

VectorXd
NodeValues::GetDerivativeOfPosWrtPhaseDuration (double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);

  auto info = polynomial_info_.at(id);
  double percent_of_phase = 1./info.num_polys_in_phase_;
  double inner_derivative = percent_of_phase;
  VectorXd vel = GetPoint(t_global).v_;
  VectorXd dxdT = cubic_polys_.at(id)->GetDerivativeOfPosWrtDuration(t_local);

  return inner_derivative*dxdT - info.poly_id_in_phase_*percent_of_phase*vel;
}

} /* namespace xpp */
