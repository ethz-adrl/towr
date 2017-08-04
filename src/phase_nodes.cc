/**
 @file    phase_nodes1.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 4, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/phase_nodes.h>

#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/bound.h>
#include <xpp/opt/variables/node_values.h>

namespace xpp {
namespace opt {


PhaseNodes::PhaseNodes (const Node& initial_value,
                        const ContactVector& contact_schedule,
                        const VecDurations& phase_durations,
                        Type type,
                        const std::string& name,
                        int n_polys_in_changing_phase)
{
  type_ = type;
  polynomial_info_ = BuildPolyInfos(contact_schedule, n_polys_in_changing_phase, type);
  VecDurations poly_durations = VecDurations(polynomial_info_.size());
  auto nodes = std::vector<Node>(poly_durations.size()+1, initial_value);
  Init(nodes, polynomial_info_, poly_durations, name);
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
