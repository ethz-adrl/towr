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

NodeValues::NodeValues (const VecTimes& times, const Node& initial_value, const std::string& id)
    :Component(-1,"NodeValues_" + id)
{
  n_dim_ = initial_value.val.at(kPos).rows();

  int n_nodes = times.size()+1;
  nodes_ = std::vector<Node>(n_nodes, initial_value);
  SetRows(n_nodes*2*n_dim_);


  for (double t : times) {
    CubicHermitePoly p(n_dim_);
    p.SetDuration(t);
    p.SetNodes(initial_value, initial_value);
    polynomials_.push_back(p);
  }

  durations_ = times;
}

NodeValues::~NodeValues ()
{
  // TODO Auto-generated destructor stub
}

VectorXd
NodeValues::GetValues () const
{
  VectorXd x(GetRows());

  for (int i=0; i<nodes_.size(); ++i)
    for (MotionDerivative d : {kPos, kVel})
      x.middleRows(Index(i, d, X), n_dim_) = nodes_.at(i).val.at(d);

  return x;
}

void
NodeValues::SetValues (const VectorXd& x)
{
  for (int i=0; i<nodes_.size(); ++i)
    for (MotionDerivative d : {kPos, kVel})
      nodes_.at(i).val.at(d) = x.middleRows(Index(i,d,X), n_dim_);

  UpdatePolynomials();
}

int
NodeValues::Index (int node, MotionDerivative deriv, int dim) const
{
  return (node+deriv)*n_dim_ + dim;
}

int
NodeValues::GetNodeId (int poly_id, Side side) const
{
  return poly_id + side;
}

Jacobian
NodeValues::GetJacobianOfPosWrtNodes (double t_global) const
{
  Jacobian jac(n_dim_, GetRows());

   int poly_id    = Spline::GetSegmentID(t_global,durations_);
   double t_local = Spline::GetLocalTime(t_global, durations_);

   // always only two nodes affect current values at all times
   for (Side side : {Side::Start, Side::End}) {
     int node = GetNodeId(poly_id,side);

     for (auto d : {kPos, kVel}) {

       double dxdp = polynomials_.at(poly_id).GetDerivativeOfPosWrt(side, d, t_local);

       // same value for x,y,z
       for (int dim=0; dim<n_dim_; ++dim)
         jac.insert(dim, Index(node, d, dim)) = dxdp;
     }
   }

  return jac;
}


void
NodeValues::UpdatePolynomials ()
{
  for (int i=0; i<polynomials_.size(); ++i) {
    polynomials_.at(i).SetNodes(nodes_.at(GetNodeId(i,Side::Start)),
                                nodes_.at(GetNodeId(i,Side::End)));
  }
}

} /* namespace opt */
} /* namespace xpp */
























