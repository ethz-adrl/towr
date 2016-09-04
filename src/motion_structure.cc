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
                                  const PhaseVec& phases)
{
  Init(start_legs, step_legs, phases);
}

MotionStructure::~MotionStructure ()
{
  // TODO Auto-generated destructor stub
}

void
MotionStructure::Init (const LegIDVec& start_legs, const LegIDVec& step_legs,
                       const PhaseVec& phases)
{
  start_stance_ = start_legs;
  steps_ = step_legs;
  phases_ = phases;
}

MotionStructure::MotionInfoVec
MotionStructure::GetContactInfoVec (double dt) const
{
  xpp::hyq::SupportPolygonContainer foothold_container;
  foothold_container.Init(start_stance_, steps_);
  auto supp = foothold_container.AssignSupportPolygonsToPhases(phases_);


  MotionInfoVec info;

  double t_global = 0;
  for (auto phase : phases_) {

    auto stance_feet = supp.at(phase.id_).GetFootholds();

    int nodes_in_phase = std::floor(phase.duration_/dt);

    for (int k=0; k<nodes_in_phase; ++k ) {

      for (const auto& f : stance_feet)
        info.push_back(MotionInfo(t_global+k*dt, f.id, f.leg));
    }

    t_global += phase.duration_;
  }

  // even though the last footstep doesn't create a support polygon, still include
  // this last time instance with contact configuration
  auto final_stance_feet = foothold_container.GetFootholds();
  for (const auto& f : final_stance_feet)
    info.push_back(MotionInfo(t_global, f.id, f.leg));

  return info;
}

MotionStructure::PhaseVec
MotionStructure::GetPhases () const
{
  return phases_;
}

} /* namespace zmp */
} /* namespace xpp */
