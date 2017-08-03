/**
 @file    node_values.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/node_values.h>

#include <array>
#include <cmath>
#include <tuple>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

NodeValues::NodeValues () : Component(-1, "node_values_placeholder")
{
}

NodeValues::~NodeValues () {}

void
NodeValues::Init (const Node& initial_value, VecDurations& poly_durations,
                  const std::string& name)
{
  PolyInfoVec poly_infos;

  for (int i=0; i<poly_durations.size(); ++i) {
    PolyInfo info;
    info.is_constant_ = false; // always use different node for start and end
    info.num_polys_in_phase_ = 1;
    info.phase_ = i;
    info.poly_id_in_phase_ = 0;

    poly_infos.push_back(info);
  }

  Init(initial_value, poly_infos, poly_durations, name);
}

void
NodeValues::Init (const Node& initial_value, const PolyInfoVec& poly_infos,
                  VecDurations& poly_durations, const std::string& name)
{
  SetName(name);
  n_dim_ = initial_value.at(kPos).rows();

  polynomial_info_ = poly_infos;
  int n_polys = polynomial_info_.size();
  poly_durations_ = poly_durations;

  nodes_.push_back(initial_value);
  for (auto& infos : poly_infos) {
    auto p = std::make_shared<PolyType>(n_dim_);
    cubic_polys_.push_back(p);
    nodes_.push_back(initial_value);
  }

  SetNodeMappings();
  int n_opt_variables = opt_to_spline_.size() * 2*n_dim_;
  SetRows(n_opt_variables);
  bounds_ = VecBound(GetRows(), kNoBound_);

  UpdatePolynomials();
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
                                 poly_durations_.at(i));
  }
}


bool
NodeValues::DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const
{
  return poly_vars == GetName();
}

const StateLinXd
NodeValues::GetPoint(double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);
  return cubic_polys_.at(id)->GetPoint(t_local);
}


Jacobian
NodeValues::GetJacobian (double t_global,  MotionDerivative dxdt) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);
  return GetJacobian(id, t_local, dxdt);
}

void
NodeValues::AddBound (int node_id, const Node& node)
{
  for (int idx=0; idx<GetRows(); ++idx)
    for (auto info : GetNodeInfo(idx))
      if (info.id_ == node_id) {
        double bound = node.at(info.deriv_)(info.dim_);
        bounds_.at(idx) = Bound(bound, bound);
      }
}

void
NodeValues::AddFinalBound (const Node& node)
{
  AddBound(nodes_.size()-1, node);
}

Jacobian
NodeValues::GetJacobian (int poly_id, double t_local, MotionDerivative dxdt) const
{
  // spring_clean_ this is very important, as at every local time,
  // different polynomials can be active depending on poly durations
  // but only if durations are optimized as well... adapt!
  Jacobian jac = Eigen::MatrixXd::Zero(n_dim_, GetRows()).sparseView(1.0, -1.0);

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







PhaseNodes::PhaseNodes (const Node& initial_value,
                        const ContactVector& contact_schedule,
                        const VecDurations& phase_durations,
                        Type type,
                        const std::string& name,
                        int n_polys_in_changing_phase)
{
  type_ = type;
  bool is_constant_during_contact = type==Motion? true : false;

  for (int i=0; i<contact_schedule.size(); ++i) {
    if (contact_schedule.at(i) == is_constant_during_contact)
      polynomial_info_.push_back(PolyInfo(i,0,1, true));
    else
      for (int j=0; j<n_polys_in_changing_phase; ++j)
        polynomial_info_.push_back(PolyInfo(i,j,n_polys_in_changing_phase, false));
  }

  VecDurations poly_durations = VecDurations(polynomial_info_.size());
  Init(initial_value, polynomial_info_, poly_durations, name);
}


PhaseNodes::~PhaseNodes ()
{
}

void
PhaseNodes::UpdateDurations(const VecDurations& phase_durations)
{
  int i=0;
  for (auto info : polynomial_info_)
    poly_durations_.at(i++) = phase_durations.at(info.phase_)/info.num_polys_in_phase_;

  UpdatePolynomials();
}

VecBound
PhaseNodes::GetBounds () const
{
  switch (type_) {
    case Force:  return OverlayForceBounds(bounds_);
    case Motion: return OverlayMotionBounds(bounds_);
    default:     assert(false); // type not defined
  }
}

VecBound
PhaseNodes::OverlayMotionBounds (VecBound bounds) const
{
  for (int idx=0; idx<GetRows(); ++idx) {
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

VecBound
PhaseNodes::OverlayForceBounds (VecBound bounds) const
{
  double max_force = 10000;
  for (int idx=0; idx<GetRows(); ++idx) {

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


} /* namespace opt */
} /* namespace xpp */
