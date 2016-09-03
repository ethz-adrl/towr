/**
 @file    stance_feet_calculator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/stance_feet_calculator.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/com_motion.h>

namespace xpp {
namespace zmp {

StanceFeetCalculator::StanceFeetCalculator ()
{
  // TODO Auto-generated constructor stub
}

StanceFeetCalculator::StanceFeetCalculator (const LegIDVec& start_legs,
                                            const LegIDVec& step_legs,
                                            const PhaseVec& phases,
                                            double dt)
{
  Init(start_legs, step_legs, phases, dt);
}

StanceFeetCalculator::~StanceFeetCalculator ()
{
  // TODO Auto-generated destructor stub
}

void
StanceFeetCalculator::Init (const LegIDVec& start_legs, const LegIDVec& step_legs,
                            const PhaseVec& phases, double dt)
{
  start_stance_ = start_legs;
  steps_ = step_legs;
  phases_ = phases;

  contact_info_vec_ = BuildContactInfoVec(dt);
}

std::vector<StanceFeetCalculator::ContactInfo>
StanceFeetCalculator::BuildContactInfoVec (double dt) const
{
  xpp::hyq::SupportPolygonContainer foothold_container;

  foothold_container.Init(start_stance_, steps_);

  auto supp = foothold_container.AssignSupportPolygonsToPhases(phases_);

  std::vector<ContactInfo> info;


  double t_global = 0;
  for (auto phase : phases_) {

    auto stance_feet = supp.at(phase.id_).GetFootholds();

    int nodes_in_phase = std::floor(phase.duration_/dt);

    for (int k=0; k<nodes_in_phase; ++k ) {

      for (const auto& f : stance_feet)
        info.push_back(ContactInfo(t_global+k*dt, f.id, f.leg));
    }

    t_global += phase.duration_;


  }

  return info;
}

std::vector<StanceFeetCalculator::ContactInfo>
StanceFeetCalculator::GetContactInfoVec () const
{
  return contact_info_vec_;
}


} /* namespace zmp */
} /* namespace xpp */

