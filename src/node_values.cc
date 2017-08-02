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

NodeValues::NodeValues () : Component(-1, "node_values_placeholder")
{
}

NodeValues::~NodeValues () {}


void
NodeValues::Init (const Node& initial_value, int n_polynomials,
                  const std::string& name)
{
  SetName(name);
  n_dim_ = initial_value.at(kPos).rows();

  nodes_.push_back(initial_value);
  for (int i=0; i<n_polynomials; ++i) {
    auto p = std::make_shared<PolyType>(n_dim_);
    cubic_polys_.push_back(p);
    nodes_.push_back(initial_value);
  }

  UpdatePolynomials();

  SetNodeMappings();


  int n_opt_variables = opt_to_spline_.size() * 2*n_dim_;
  SetRows(n_opt_variables);
}

void
NodeValues::SetNodeMappings ()
{
  // every optimization value maps to a different node
  int opt_id = 0;
  for (int node_id=0; node_id<nodes_.size(); ++node_id)
    opt_to_spline_[opt_id++] = {node_id};
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


// zmp_ remove the constant here, this is nonsense
void
NodeValues::UpdatePolynomials () const
{
  for (int i=0; i<cubic_polys_.size(); ++i) {
    cubic_polys_.at(i)->SetNodes(nodes_.at(GetNodeId(i,Side::Start)),
                                 nodes_.at(GetNodeId(i,Side::End)),
                                 times_.at(i));
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
  std::tie(id, t_local) = GetLocalTime(t_global, times_);
  return cubic_polys_.at(id)->GetPoint(t_local);
}


Jacobian
NodeValues::GetJacobian (double t_global,  MotionDerivative dxdt) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, times_);
  return GetJacobian(id, t_local, dxdt);
}

Jacobian
NodeValues::GetJacobian (int poly_id, double t_local, MotionDerivative dxdt) const
{
  // spring_clean_ this is very important, as at every local time,
  // different polynomials can be active depending on poly durations
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
PhaseNodes::GetDerivativeOfPosWrtPhaseDuration (double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, times_);

  // polynomial durations derived w.r.t. opt. times
  int phase_id, id_local, num_polys_in_phase;
  bool is_constant;
  std::tie(phase_id, id_local, num_polys_in_phase, is_constant) = percent_of_phase_.at(id);


  double percent_of_phase = 1./num_polys_in_phase;
  double inner_derivative = percent_of_phase;
  VectorXd vel = GetPoint(t_global).v_;

  VectorXd tune = id_local*percent_of_phase*vel;

  return inner_derivative*cubic_polys_.at(id)->GetDerivativeOfPosWrtDuration(t_local) - tune;
}






PhaseNodes::PhaseNodes (const Node& initial_value,
                        const SchedulePtr& contact_schedule,
                        const std::string& name,
                        bool is_constant_during_contact,
                        int n_polys_in_changing_phase)
{
  SetTimeStructure(contact_schedule->GetPhaseCount(),
                   is_constant_during_contact,
                   contact_schedule,
                   n_polys_in_changing_phase);

  Init(initial_value, percent_of_phase_.size(), name);
}


PhaseNodes::~PhaseNodes ()
{
}


void
PhaseNodes::SetTimeStructure (int num_phases,
                              bool is_constant_during_contact,
                              const SchedulePtr& contact_schedule,
                              int polys_in_non_constant_phase)
{
  for (int i=0; i<num_phases; ++i) {
    if (contact_schedule->GetContact(i) == is_constant_during_contact)
      percent_of_phase_.push_back(std::make_tuple(i,0,1, true));
    else
      for (int j=0; j<polys_in_non_constant_phase; ++j)
        percent_of_phase_.push_back(std::make_tuple(i,j,polys_in_non_constant_phase, false));
  }

  times_ = VecTimes(percent_of_phase_.size());
}


void
PhaseNodes::SetNodeMappings ()
{
  int opt_id = 0;
  for (int i=0; i<percent_of_phase_.size(); ++i) {
    int node_id_start = GetNodeId(i, CubicHermitePoly::Start);

    opt_to_spline_[opt_id].push_back(node_id_start);
    bool is_constant_poly = std::get<3>(percent_of_phase_.at(i));
    // use same value for next node if polynomial is constant
    if (!is_constant_poly)
      opt_id++;
  }

  opt_to_spline_[opt_id].push_back(nodes_.size()-1); // add last node
}


void
PhaseNodes::UpdateTimes(const VecTimes& durations)
{
  int i=0;
  for (auto info : percent_of_phase_) {
    int phase = std::get<0>(info);
    int n_polys_in_phase = std::get<2>(info);
    times_.at(i++) = durations.at(phase)/n_polys_in_phase;
  }
}


EEMotionNodes::EEMotionNodes (const Node& initial_value,
                              const SchedulePtr& contact_schedule,
                              int splines_per_swing_phase,
                              int ee)
    :PhaseNodes(initial_value,
                contact_schedule,
                id::GetEEId(ee),
                true,
                splines_per_swing_phase)
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
                              const SchedulePtr& contact_schedule,
                              int splines_per_stance_phase,
                              int ee)
    :PhaseNodes(initial_force,
                contact_schedule,
                id::GetEEForceId(ee),
                false,
                splines_per_stance_phase)
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



} /* namespace opt */
} /* namespace xpp */
