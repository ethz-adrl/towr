/**
 @file    node_values.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/node_values.h>

#include <Eigen/Dense>

#include <xpp/opt/variables/spline.h>

namespace xpp {
namespace opt {

NodeValues::NodeValues (const Node& initial_value,
                        const VecTimes& times,
                        const std::string& name)
    :Component(-1, name)
{
  n_dim_ = initial_value.at(kPos).rows();


//  int var_max = 1000;
//  int n_var = 0;
//  for (int idx=0; idx<var_max; ++idx) {
//    for (auto n : GetNodeInfo(idx)) {
//      if (n.id_==n_nodes-1 && n.deriv_==kVel && n.dim_==n_dim_-1)
//        n_var = idx+1;
//    }
//  }


//  NodeInfo info;
//  info.id_ = n_nodes-1;
//  info.deriv_ = kVel,
//  info.dim_ = n_dim_-1;
//  int n_var = Index(info)+1; // last node, last dimension
//  n_var += timings_.size(); // zmp_ optimize over these as well


  nodes_.push_back(initial_value);
  for (double t : times) {

    auto p = std::make_shared<PolyType>(n_dim_);
    p->SetNodes(initial_value, initial_value, t);

    cubic_polys_.push_back(p);
    nodes_.push_back(initial_value);
    timings_.push_back(t);

  }


//  int n_nodes = timings_.size()+1;
//  nodes_ = std::vector<Node>(n_nodes, initial_value);

  int n_opt_variables = 2*n_dim_*nodes_.size();
  SetRows(n_opt_variables); // because two consecutive nodes are the same
}


std::vector<NodeValues::NodeInfo>
NodeValues::GetNodeInfo (int idx) const
{
  std::vector<NodeInfo> nodes;


//  // always two consecutive node pairs are equal
//  int n_opt_values_per_node_ = 2*n_dim_;
//  int opt_node = std::floor(idx/n_opt_values_per_node_);
//  int internal_id = idx%n_opt_values_per_node_; // 0...6
//
//  // every idx maps to two nodes
//  NodeInfo node;
//  node.deriv_ = static_cast<MotionDerivative>(std::floor(internal_id/n_dim_));
//  node.dim_   = internal_id-node.deriv_*n_dim_;
//
//  for (int i=0; i<2; ++i) {
//    node.id_ = 2*opt_node + i;
//    nodes.push_back(node);
//  }





//  // all velocities are left at zero, only optimizing positions
//  // every second foothold node position is equal
//  int n_opt_values_per_node = n_dim_;
//  int node_bar = std::floor(idx/n_opt_values_per_node);
//
//
//  NodeInfo node;
////  node.id_    = std::floor(idx/n_opt_values_per_node_);
//  node.deriv_ = kPos;
//  node.dim_   = idx%n_opt_values_per_node;
////  nodes.push_back(node);
//
//  for (int i=0; i<2; ++i) {
//    node.id_ = 2*node_bar + i;
//    nodes.push_back(node);
//  }









  // every value of every node gets its own optimization variable
  NodeInfo node;
  int n_opt_values_per_node_ = 2*n_dim_;
  node.id_    = std::floor(idx/n_opt_values_per_node_);

  int internal_id = idx%n_opt_values_per_node_; // 0...6
  node.deriv_ = (MotionDerivative)std::floor(internal_id/n_dim_); // 0 for 0,1,2 and 1 for 3,4,5
  node.dim_   = internal_id-node.deriv_*n_dim_;

  nodes.push_back(node);



  return nodes;
}



NodeValues::~NodeValues () {}

VectorXd
NodeValues::GetValues () const
{
  VectorXd x(GetRows());

  for (int idx=0; idx<x.rows(); ++idx) {
    for (auto info : GetNodeInfo(idx))
      x(idx) = nodes_.at(info.id_).at(info.deriv_)(info.dim_);
  }

  return x;
}

void
NodeValues::SetValues (const VectorXd& x)
{
  for (int idx=0; idx<x.rows(); ++idx) {
    for (auto info : GetNodeInfo(idx))
      nodes_.at(info.id_).at(info.deriv_)(info.dim_) = x(idx);
  }

  UpdatePolynomials();
}

//VecBound
//NodeValues::GetBounds () const
//{
//  VecBound bounds(GetRows());
//
//  int row=0;
//
//  for (int i=0; i<nodes_.size(); ++i)
//    for (MotionDerivative d : {kPos, kVel})
//      for (int dim=0; dim<n_dim_; ++dim)
//        bounds.at(Index(i,d, dim)) = kNoBound_;
//
//  int timings_start = bounds.size() - timings_.size();
//  for (int i=0; i<timings_.size(); ++i) {
//    bounds.at(timings_start+i) = Bound(0.1, 0.4);
//  }
//
//  return bounds;
//}

//int
//NodeValues::Index (NodeInfo info) const
//{
////  Node
////
//  // zmp_ inefficient, but no code duplication, hm....
//  for (int idx=0; idx<GetRows(); ++idx)
//    for (NodeInfo i : GetNodeInfo(idx))
//      if (i == info)
//        return idx;
//
//
//  // could also be that value is fixed and therefore has no index
//
//
//
////  // results in same position and velocity of pairwise nodes
////  // e.g. keeping foot on ground for this duration
////  int opt_node = std::floor(info.id_/2); // node 0 and 1 -> 0
////                                     // node 2 and 3 -> 1
////
////
////  // change only the positions
//////  return node*n_dim_ + dim;
////
////
////  return opt_node*2*n_dim_ + info.deriv_*n_dim_ + info.dim_;
//}



void
NodeValues::UpdatePolynomials ()
{
  for (int i=0; i<cubic_polys_.size(); ++i) {
    cubic_polys_.at(i)->SetNodes(nodes_.at(GetNodeId(i,Side::Start)),
                                 nodes_.at(GetNodeId(i,Side::End)),
                                 timings_.at(i));
  }
}

Jacobian
NodeValues::GetJacobian (int poly_id, double t_local) const
{
  Jacobian jac(n_dim_, GetRows());

  for (int idx=0; idx<jac.cols(); ++idx) {
    for (NodeInfo info : GetNodeInfo(idx)) {
      for (Side side : {Side::Start, Side::End}) {

        int node = GetNodeId(poly_id,side);

        if (node == info.id_) {
          double val = cubic_polys_.at(poly_id)->GetDerivativeOfPosWrt(side, info.deriv_, t_local);
          jac.coeffRef(info.dim_, idx) += val;
        }
      }
    }
  }

  return jac;
}

int
NodeValues::GetNodeId (int poly_id, Side side) const
{
  return poly_id + side;
}





HermiteSpline::HermiteSpline (const OptVarsPtr& opt_vars,
                              const std::string& node_id)
{
  node_values_ = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(node_id));
  auto v = node_values_->GetCubicPolys();
  polynomials_.assign(v.begin(), v.end()); // links the two
}


HermiteSpline::~HermiteSpline() {};

bool
HermiteSpline::DoVarAffectCurrentState (const std::string& poly_vars,
                                        double t_current) const
{
  return poly_vars == node_values_->GetName();
}

Jacobian
HermiteSpline::GetJacobian (double t_global,  MotionDerivative dxdt) const
{
  assert(dxdt == kPos); // derivative of velocity/acceleration not yet implemented
//  UpdateDurations();

  int poly_id     = GetSegmentID(t_global);
  double t_local  = GetLocalTime(t_global); // these are both wrong when adding extra polynomial

  return node_values_->GetJacobian(poly_id, t_local);
}






} /* namespace opt */
} /* namespace xpp */
