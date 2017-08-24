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

  // node is considered contact node if either left or right polynomial
  // represents a contact phase
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



EndeffectorNodes::EndeffectorNodes (int n_dim,
                                  const ContactVector& contact_schedule,
                                  const std::string& name,
                                  int n_polys)
    :PhaseNodes(n_dim, contact_schedule, name, n_polys, Motion)
{
}

EndeffectorNodes::~EndeffectorNodes ()
{
}

bool
EndeffectorNodes::IsContactNode (int node_id) const
{
  return IsConstantNode(node_id);
}

VecBound
EndeffectorNodes::GetBounds () const
{
  for (int idx=0; idx<GetRows(); ++idx) {

    auto node = GetNodeInfo(idx).front(); // bound idx by first node it represents

    // endeffector is not allowed to move if in stance phase
    if (IsContactNode(node.id_))
      if (node.deriv_ == kVel)
        bounds_.at(idx) = kEqualityBound_;
  }

  return bounds_;
}



ForceNodes::ForceNodes (int n_dim, const ContactVector& contact_schedule,
                        const std::string& name, int n_polys,
                        double force_max)
    :PhaseNodes(n_dim, contact_schedule, name, n_polys, Force)
{
  f_max_ = force_max;
}

ForceNodes::~ForceNodes ()
{
}

bool
ForceNodes::IsSwingNode (int node_id) const
{
  return IsConstantNode(node_id);
}

VecBound
ForceNodes::GetBounds () const
{
  for (int idx=0; idx<GetRows(); ++idx) {

    NodeInfo n0 = GetNodeInfo(idx).front(); // only one node anyway
    bool is_swing = IsSwingNode(n0.id_);

    if (is_swing)
      bounds_.at(idx) = kEqualityBound_; // force must be zero
    else { // stance-phase -> forces can be applied


      if (n0.deriv_ == kPos) {

        if (n0.dim_ == X || n0.dim_ == Y)
          bounds_.at(idx) = Bound(-f_max_, f_max_);

        // unilateral contact forces ("pulling" on ground not possible)
        if (n0.dim_ == Z)
          bounds_.at(idx) = Bound(0.0, f_max_);
      }

      if (n0.deriv_ == kVel && n0.dim_ == Z) {
        bounds_.at(idx) = kEqualityBound_; // zero slope to never exceed zero force
      }

    }
  }

  return bounds_;
}

} /* namespace opt */
} /* namespace xpp */


