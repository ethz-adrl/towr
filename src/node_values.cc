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

NodeValues::NodeValues (bool is_motion,
                        const Node& initial_value,
                        const VecTimes& times,
                        const std::string& name)
    :Component(-1, name)
{
  n_dim_ = initial_value.at(kPos).rows();

  bool is_start_multi_poly_ = is_motion? false : true;

  bool single_poly_phase = !is_start_multi_poly_;


  int n_polys_per_multi_poly_phase = 2;
  nodes_.push_back(initial_value);

  for (double T : times) {

    if (single_poly_phase) {

      auto p = std::make_shared<PolyType>(n_dim_);
      cubic_polys_.push_back(p);
      nodes_.push_back(initial_value);
      timings_.push_back(T);

    } else { // multip_poly_phase (=swing_phase for ee_motion and stance_phase for ee_force)

      for (int i=0; i<n_polys_per_multi_poly_phase; ++i) {

        auto p = std::make_shared<PolyType>(n_dim_);

        cubic_polys_.push_back(p);
        nodes_.push_back(initial_value);
        timings_.push_back(T/n_polys_per_multi_poly_phase);


      }


    }

    single_poly_phase = !single_poly_phase; // make multi poly phase

  }

  UpdatePolynomials();




  int single_poly_node_in_cycle = is_start_multi_poly_*n_polys_per_multi_poly_phase;

  int opt_id = 0;
  for (int node_id=0; node_id<nodes_.size(); ++node_id) {

    opt_to_spline_[opt_id].push_back(node_id);

    int node_in_cycle = node_id%(n_polys_per_multi_poly_phase+1);

    // use same optimization variable for next node in single poly phase
    if (node_in_cycle != single_poly_node_in_cycle)
      opt_id++;
  }


  int n_opt_variables = opt_to_spline_.size() * 2*n_dim_;
  SetRows(n_opt_variables);
}


std::vector<NodeValues::NodeInfo>
NodeValues::GetNodeInfo (int idx) const
{
  std::vector<NodeInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*n_dim_;
  int opt_node = std::floor(idx/n_opt_values_per_node_);
  int internal_id = idx%n_opt_values_per_node_; // 0...6

  NodeInfo node;
  node.deriv_ = static_cast<MotionDerivative>(std::floor(internal_id/n_dim_));
  node.dim_   = internal_id-node.deriv_*n_dim_;

  for (auto node_id : opt_to_spline_.at(opt_node)) {
    node.id_ = node_id;
    nodes.push_back(node);
  }

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
//  VecBound bounds(GetRows(), kNoBound_);
//
//  int row=0;
//
//
//  for (int idx=0; idx<bounds.size(); ++idx) {
//    for (auto info : GetNodeInfo(idx)) {
//      if (info.deriv_ == kVel) {
//        bounds.at(idx) = kEqualityBound_;
//      }
//    }
//  }
//
//
//
////  int timings_start = bounds.size() - timings_.size();
////  for (int i=0; i<timings_.size(); ++i) {
////    bounds.at(timings_start+i) = Bound(0.1, 0.4);
////  }
//
//  return bounds;
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

  int poly_id     = GetSegmentID(t_global);
  double t_local  = GetLocalTime(t_global); // these are both wrong when adding extra polynomial

  return node_values_->GetJacobian(poly_id, t_local);
}






} /* namespace opt */
} /* namespace xpp */
