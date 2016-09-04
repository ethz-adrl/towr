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
  // TODO Auto-generated constructor stub
}

MotionStructure::MotionStructure (const LegIDVec& start_legs,
                                  const LegIDVec& step_legs,
                                  const PhaseVec& phases,
                                  double dt)
{
  Init(start_legs, step_legs, phases, dt);
}

MotionStructure::~MotionStructure ()
{
  // TODO Auto-generated destructor stub
}

void
MotionStructure::Init (const LegIDVec& start_legs, const LegIDVec& step_legs,
                       const PhaseVec& phases, double dt)
{
  start_stance_ = start_legs;
  steps_        = step_legs;
  phases_       = phases;
  dt_           = dt;
}

MotionStructure::MotionInfoVec
MotionStructure::GetContactInfoVec () const
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

      for (const auto& f : stance_feet) {
        contact_info.foothold_ids_.push_back(f.id);
        contact_info.legs_.push_back(f.leg);
      }

      info.push_back(contact_info);
    }

    t_global += phase.duration_;
  }

  // even though the last footstep doesn't create a support polygon, still include
  // this last time instance with contact configuration
  MotionInfo final_contact;
  final_contact.time_ = t_global;
  for (const auto& f : foothold_container.GetFinalFootholds()) {
    final_contact.foothold_ids_.push_back(f.id);
    final_contact.legs_.push_back(f.leg);
  }

  info.push_back(final_contact);

  return info;
}

int
MotionStructure::GetTotalNumberOfDiscreteContacts () const
{
  auto contact_info_vec = GetContactInfoVec();

  int i = 0;
  for (auto node : contact_info_vec)
    i += node.foothold_ids_.size();

  return i;
}

MotionStructure::PhaseVec
MotionStructure::GetPhases () const
{
  return phases_;
}

} /* namespace zmp */
} /* namespace xpp */
