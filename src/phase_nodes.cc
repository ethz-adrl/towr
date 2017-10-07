/**
 @file    phase_nodes1.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 4, 2017
 @brief   Brief description
 */

#include <xpp/variables/phase_nodes.h>

#include <xpp/cartesian_declarations.h>
#include <xpp/variables/node_values.h>

namespace xpp {
namespace opt {


PhaseNodes::PhaseNodes (int n_dim,
                        const ContactVector& contact_schedule,
                        const std::string& name,
                        int n_polys_in_changing_phase,
                        Type type)
    :NodeValues(n_dim,
                BuildPolyInfos(contact_schedule, n_polys_in_changing_phase, type),
                name)
{
}

PhaseNodes::PolyInfoVec
PhaseNodes::BuildPolyInfos (const ContactVector& contact_schedule,
                            int n_polys_in_changing_phase,
                            Type type) const
{
  PolyInfoVec polynomial_info;

  bool is_constant_during_contact = type==Motion? true : false;

  for (int i=0; i<contact_schedule.size(); ++i) {
    if (contact_schedule.at(i) == is_constant_during_contact)
      polynomial_info.push_back(PolyInfo(i,0,1, true));
    else
      for (int j=0; j<n_polys_in_changing_phase; ++j)
        polynomial_info.push_back(PolyInfo(i,j,n_polys_in_changing_phase, false));
  }

  return polynomial_info;
}

PhaseNodes::~PhaseNodes ()
{
}

void
PhaseNodes::UpdateDurations(const VecDurations& phase_durations)
{
  durations_change_ = true;
  poly_durations_ = ConvertPhaseToSpline(phase_durations);
  UpdatePolynomials();
}

void
PhaseNodes::InitializeVariables (const VectorXd& initial_pos,
                                 const VectorXd& final_pos,
                                 const VecDurations& phase_durations)
{
  NodeValues::InitializeVariables(initial_pos,
                                  final_pos,
                                  ConvertPhaseToSpline(phase_durations));
}

bool
PhaseNodes::IsConstantNode (int node_id) const
{
  bool is_constant = false;

  // node is considered constant if either left or right polynomial
  // belongs to a constant phase
  for (int poly_id : GetAdjacentPolyIds(node_id))
    if (polynomial_info_.at(poly_id).is_constant_)
      is_constant = true;

  return is_constant;
}

PhaseNodes::VecDurations
PhaseNodes::ConvertPhaseToSpline (const VecDurations& phase_durations) const
{
  VecDurations spline_durations;

  for (auto info : polynomial_info_)
    spline_durations.push_back(phase_durations.at(info.phase_)/info.num_polys_in_phase_);

  return spline_durations;
}

int
PhaseNodes::GetPolyIDAtStartOfPhase (int phase) const
{
  int poly_id=0;
  for (int i=0; i<polynomial_info_.size(); ++i)
    if (polynomial_info_.at(i).phase_ == phase)
      return i;
}

Vector3d
PhaseNodes::GetValueAtStartOfPhase (int phase) const
{
  int poly_id=GetPolyIDAtStartOfPhase(phase);
  return cubic_polys_.at(poly_id)->GetPoint(0.0).p_;
}

int
PhaseNodes::GetNodeIDAtStartOfPhase (int phase) const
{
  int poly_id=GetPolyIDAtStartOfPhase(phase);
  return GetNodeId(poly_id, Side::Start);
}




EEMotionNodes::EEMotionNodes (const ContactVector& contact_schedule,
                              const std::string& name,
                              int n_polys)
    :PhaseNodes(kDim3d, contact_schedule, name, n_polys, Motion)
{
}

EEMotionNodes::~EEMotionNodes ()
{
}

bool
EEMotionNodes::IsContactNode (int node_id) const
{
  return IsConstantNode(node_id);
}

VecBound
EEMotionNodes::GetBounds () const
{
  for (int idx=0; idx<GetRows(); ++idx) {

    auto node = GetNodeInfo(idx).front(); // bound idx by first node it represents





    // endeffector is not allowed to move if in stance phase
    if (IsContactNode(node.id_)) {
      if (node.deriv_ == kVel)
        bounds_.at(idx) = BoundZero;
    }
    else { // node in pure swing-phase
      if (node.deriv_ == kVel && node.dim_ == Z)
        bounds_.at(idx) = BoundZero; // zero velocity at top
    }





  }

  return bounds_;
}






EEForceNodes::EEForceNodes (const ContactVector& contact_schedule,
                        const std::string& name, int n_polys)
    :PhaseNodes(kDim3d, contact_schedule, name, n_polys, Force)
{
}

EEForceNodes::~EEForceNodes ()
{
}

bool
EEForceNodes::IsStanceNode (int node_id) const
{
  return !IsConstantNode(node_id);
}

int
EEForceNodes::GetPhase (int node_id) const
{
  assert(IsStanceNode(node_id)); // because otherwise it has two phases

  int poly_id = GetAdjacentPolyIds(node_id).front();
  return polynomial_info_.at(poly_id).phase_;
}

VecBound
EEForceNodes::GetBounds () const
{
  for (int idx=0; idx<GetRows(); ++idx) {

    NodeInfo n0 = GetNodeInfo(idx).front(); // only one node anyway

    if (IsStanceNode(n0.id_)) {

//      if (n0.deriv_ == kPos) {
//
//        if (n0.dim_ == X || n0.dim_ == Y)
//          bounds_.at(idx) = Bound(-f_max_, f_max_);
//
//        // unilateral contact forces ("pulling" on ground not possible)
//        if (n0.dim_ == Z)
//          bounds_.at(idx) = Bound(0.0, f_max_);
//      }
//
//      if (n0.deriv_ == kVel) {
//        bounds_.at(idx) = BoundZero; // zero slope to never exceed zero force between nodes
//      }

    } else { // swing node
      bounds_.at(idx) = BoundZero; // force must be zero
    }

  }

  return bounds_;
}

} /* namespace opt */
} /* namespace xpp */


