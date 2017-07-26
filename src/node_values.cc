/**
 @file    node_values.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/node_values.h>

#include <Eigen/Dense>

#include <xpp/opt/variables/spline.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

NodeValues::NodeValues () : Component(-1, "dummy")
{
}

NodeValues::~NodeValues () {}


void
NodeValues::Init (const Node& initial_value, const VecTimes& times,
                  const std::string& name)
{
  SetName(name);
  n_dim_ = initial_value.at(kPos).rows();

  nodes_.push_back(initial_value);
  for (double T : times) {
    auto p = std::make_shared<PolyType>(n_dim_);
    cubic_polys_.push_back(p);
    nodes_.push_back(initial_value);
    timings_.push_back(T);
  }

  UpdatePolynomials();

  // every optimization value maps to a different node
  int opt_id = 0;
  for (int node_id=0; node_id<nodes_.size(); ++node_id)
    opt_to_spline_[opt_id++] = {node_id};


  int n_opt_variables = opt_to_spline_.size() * 2*n_dim_;
  SetRows(n_opt_variables);
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
NodeValues::SetValues (const VectorXd& x)
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
                                 timings_.at(i));
  }
}

Jacobian
NodeValues::GetJacobian (int poly_id, double t_local, MotionDerivative dxdt) const
{
  Jacobian jac(n_dim_, GetRows());

  for (int idx=0; idx<jac.cols(); ++idx) {
    for (NodeInfo info : GetNodeInfo(idx)) {
      for (Side side : {Side::Start, Side::End}) {

        int node = GetNodeId(poly_id,side);

        if (node == info.id_) {
          double val = cubic_polys_.at(poly_id)->GetDerivativeOf(dxdt, side, info.deriv_, t_local);
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







PhaseNodes::PhaseNodes (const Node& initial_value, const VecTimes& phase_times,
                        const std::string& name, bool is_first_phase_constant,
                        int n_polys_in_changing_phase)
{

  VecTimes poly_times;


  bool is_constant_phase = is_first_phase_constant;
  for (double T : phase_times) {

    if (is_constant_phase)
      poly_times.push_back(T);
    else
      for (int i=0; i<n_polys_in_changing_phase; ++i)
        poly_times.push_back(T/n_polys_in_changing_phase);

    is_constant_phase = !is_constant_phase;
  }


  Init(initial_value, poly_times, name);





  int first_constant_node_in_cycle = (!is_first_phase_constant)*n_polys_in_changing_phase;

  int opt_id = 0;
  opt_to_spline_.clear();
  for (int node_id=0; node_id<nodes_.size(); ++node_id) {

    opt_to_spline_[opt_id].push_back(node_id);

    int node_in_cycle = node_id%(n_polys_in_changing_phase+1);

    // use same optimization variable for next node in single poly phase
    if (node_in_cycle != first_constant_node_in_cycle)
      opt_id++;
  }


  int n_opt_variables = opt_to_spline_.size() * 2*n_dim_;
  SetRows(n_opt_variables);
}

PhaseNodes::~PhaseNodes ()
{
}












EEMotionNodes::EEMotionNodes (const Node& initial_value,
                              const VecTimes& times,
                              int splines_per_swing_phase,
                              int ee)
    :PhaseNodes(initial_value, times, id::GetEEId(ee), true, splines_per_swing_phase)
{
}

EEMotionNodes::~EEMotionNodes ()
{
}

VecBound
EEMotionNodes::GetBounds () const
{
  VecBound bounds(GetRows(), Bound(kNoBound_));


  for (int idx=0; idx<bounds.size(); ++idx) {

    // no force allowed during swingphase
    bool is_stance = GetNodeInfo(idx).size() == 2;

    if (is_stance) {
      if (GetNodeInfo(idx).at(0).deriv_ == kVel)
        bounds.at(idx) = kEqualityBound_;

      if (GetNodeInfo(idx).at(0).dim_ == Z)
        bounds.at(idx) = kEqualityBound_; // ground is at zero height

    }


  }

  return bounds;
}





EEForcesNodes::EEForcesNodes (const Node& initial_force,
                              const VecTimes& times,
                              int splines_per_stance_phase,
                              int ee)
    :PhaseNodes(initial_force, times, id::GetEEForceId(ee), false, splines_per_stance_phase)
{
}

EEForcesNodes::~EEForcesNodes ()
{
}

VecBound
EEForcesNodes::GetBounds () const
{
  double max_force = 10000;
  VecBound bounds(GetRows(), kNoBound_);

  for (int idx=0; idx<bounds.size(); ++idx) {



    // no force or force velocity allowed during swingphase
    bool is_swing_phase = GetNodeInfo(idx).size() == 2;
    if (is_swing_phase)
      bounds.at(idx) = kEqualityBound_; // position and velocity must be zero
    else { // stance-phase -> forces can be applied

      NodeInfo n0 = GetNodeInfo(idx).front(); // only one node anyway

      if (n0.deriv_ == kPos) {

        if (n0.dim_ == X || n0.dim_ == Y)
          bounds.at(idx) = Bound(-max_force, max_force);

        // unilateral contact forces ("pulling" on ground not possible)
        if (n0.dim_ == Z)
          bounds.at(idx) = Bound(0.0, max_force);
      }

      if (n0.deriv_ == kVel && n0.dim_ == Z) {
        bounds.at(idx) = kEqualityBound_; // zero slope to never exceed maximum height
      }

    }
  }

  return bounds;
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
  int poly_id     = GetSegmentID(t_global);
  double t_local  = GetLocalTime(t_global); // these are both wrong when adding extra polynomial

  return node_values_->GetJacobian(poly_id, t_local, dxdt);
}



} /* namespace opt */
} /* namespace xpp */


