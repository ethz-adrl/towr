/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the MotionStructure class.
 */

#include <xpp/zmp/motion_structure.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/phase_info.h>

namespace xpp {
namespace zmp {

MotionStructure::MotionStructure ()
{
  dt_ = 0.2; //standart discretization
}

MotionStructure::~MotionStructure ()
{
  // TODO Auto-generated destructor stub
}

void
MotionStructure::Init (const StartStance& start_stance,
                       const LegIDVec& step_legs,
                       double t_swing, double t_stance,
                       bool insert_initial_stance,
                       bool insert_final_stance)
{
  phases_ = BuildPhases(step_legs.size(), t_swing, t_stance, insert_initial_stance,
                        insert_final_stance);
  start_stance_ = start_stance;
  steps_ = step_legs;
  cache_needs_updating_ = true;
}

MotionStructure::PhaseVec
MotionStructure::BuildPhases (int steps, double t_swing, double t_stance,
                              bool insert_init, bool insert_final) const
{
  PhaseVec phases;

  int id = 0;
  int n_completed_steps = 0;

  if (insert_init) {
    PhaseInfo phase(PhaseInfo::kStancePhase, n_completed_steps, id++, t_stance);
    phases.push_back(phase);
  }

  // steps
  for (int step=0; step<steps; ++step) {
    PhaseInfo phase(PhaseInfo::kStepPhase, n_completed_steps++, id++, t_swing);
    phases.push_back(phase);
  }

  if (insert_final) {
    PhaseInfo phase(PhaseInfo::kStancePhase, n_completed_steps, id++, 0.55);
    phases.push_back(phase);
  }

  return phases;
}

PhaseInfo
MotionStructure::GetCurrentPhase (double t_global) const
{
  double t = 0;
  for (const auto& phase: phases_) {
    t += phase.duration_;

    if (t >= t_global) // at junctions, returns previous phase (=)
      return phase;
  }
  assert(false); // this should never be reached
}

double
MotionStructure::GetTotalTime() const
{
  double T = 0.0;
  for (const auto& phase: phases_)
    T += phase.duration_;
  return T;
}


MotionStructure::MotionInfoVec
MotionStructure::GetContactInfoVec () const
{
  if (cache_needs_updating_) {
    cached_motion_vector_ = CalcContactInfoVec();
    cache_needs_updating_ = false;
  }

  return cached_motion_vector_;
}

MotionStructure::MotionInfoVec
MotionStructure::CalcContactInfoVec () const
{
  xpp::hyq::SupportPolygonContainer foothold_container;
  foothold_container.Init(start_stance_, steps_);
  auto supp = foothold_container.AssignSupportPolygonsToPhases(phases_);

  MotionInfoVec info;

  double t_global = 0;
  for (auto phase : phases_) {

    auto stance_feet = supp.at(phase.id_).GetFootholds();

    int nodes_in_phase = std::floor(phase.duration_/dt_);

    for (int k=0; k<nodes_in_phase; ++k ) {

      MotionInfo contact_info;
      contact_info.time_ = t_global+k*dt_;

      for (const auto& f : stance_feet)
        contact_info.phase_.contacts_.push_back(Contact(f.id, static_cast<EndeffectorID>(f.leg)));

      info.push_back(contact_info);
    }

    t_global += phase.duration_;
  }

  // even though the last footstep doesn't create a support polygon, still include
  // this last time instance with contact configuration
  MotionInfo final_contacts;
  final_contacts.time_ = t_global;
  for (const auto& f : foothold_container.GetFinalFootholds())
    final_contacts.phase_.contacts_.push_back(Contact(f.id, static_cast<EndeffectorID>(f.leg)));

  info.push_back(final_contacts);

  return info;
}

int
MotionStructure::GetTotalNumberOfDiscreteContacts () const
{
  auto contact_info_vec = GetContactInfoVec();

  int i = 0;
  for (auto node : contact_info_vec)
    i += node.phase_.contacts_.size();

  return i;
}

MotionStructure::PhaseVec
MotionStructure::GetPhases () const
{
  return phases_;
}

void
MotionStructure::SetDisretization (double dt)
{
  dt_ = dt;
  cache_needs_updating_ = true;
}

} /* namespace zmp */
} /* namespace xpp */

