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

  int n_nodes = times.size()+1;
  nodes_ = std::vector<Node>(n_nodes, initial_value);
  SetRows(n_nodes*2*n_dim_);

  for (double t : times) {
    auto p = std::make_shared<PolyType>(n_dim_);
    p->SetNodes(initial_value, initial_value, t);
    cubic_polys_.push_back(p);
  }

  timings_ = times;
}

NodeValues::~NodeValues () {}

VectorXd
NodeValues::GetValues () const
{
  VectorXd x(GetRows());

  for (int i=0; i<nodes_.size(); ++i)
    for (MotionDerivative d : {kPos, kVel})
      x.middleRows(Index(i,d, X), n_dim_) = nodes_.at(i).at(d);

  return x;
}

void
NodeValues::SetValues (const VectorXd& x)
{
  for (int i=0; i<nodes_.size(); ++i)
    for (MotionDerivative d : {kPos, kVel})
      nodes_.at(i).at(d) = x.middleRows(Index(i,d,X), n_dim_);

  UpdatePolynomials(timings_);
}

void
NodeValues::UpdatePolynomials (const VecTimes& durations)
{
  for (int i=0; i<cubic_polys_.size(); ++i) {
    cubic_polys_.at(i)->SetNodes(nodes_.at(GetNodeId(i,Side::Start)),
                                 nodes_.at(GetNodeId(i,Side::End)),
                                 durations.at(i));
  }
}

Jacobian
NodeValues::GetJacobian (int poly_id, double t_local, double T) const
{
  Jacobian jac(n_dim_, GetRows());

   // always only two nodes affect current values at all times
  for (Side side : {Side::Start, Side::End}) {
    int node = GetNodeId(poly_id,side);

    for (auto deriv : {kPos, kVel}) {
      double dxdp = cubic_polys_.at(poly_id)->GetDerivativeOfPosWrt(side, deriv, t_local, T);

      // same value for x,y,z
      for (int dim=0; dim<n_dim_; ++dim)
        jac.coeffRef(dim, Index(node, deriv, dim)) = dxdp;

    }
  }

  return jac;
}

int
NodeValues::Index (int node, MotionDerivative d, int dim) const
{
  return node*2*n_dim_ + d*n_dim_ + dim;
}

int
NodeValues::GetNodeId (int poly_id, Side side) const
{
  return poly_id + side;
}




HermiteSpline::HermiteSpline (const OptVarsPtr& opt_vars,
                              const std::string& node_id,
                              const VecTimes& poly_durations)
{
  durations_   = poly_durations;
  node_values_ = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(node_id));

  // zmp_ remove
//  for (auto& p : node_values_->GetCubicPolys()) {
//    polynomials_.push_back(p);
//  }

  auto v = node_values_->GetCubicPolys();
  polynomials_.assign(v.begin(), v.end()); // zmp_ links the two?
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
  double t_local  = GetLocalTime(t_global);
  double duration = durations_.at(poly_id);

  return node_values_->GetJacobian(poly_id, t_local, duration);
}






} /* namespace opt */
} /* namespace xpp */
