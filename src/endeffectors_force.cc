/**
 @file    endeffector_load.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/endeffectors_force.h>

#include <cmath>
#include <vector>

#include <xpp/bound.h>

namespace xpp {
namespace opt {


EndeffectorsForce::EndeffectorsForce (double dt, const ContactSchedule& contact_schedule)
    :Composite("endeffector_force", true)
{
  ee_ordered_ = contact_schedule.IsInContact(0.0).GetEEsOrdered();

  for (auto ee : ee_ordered_) {
    auto ee_force = std::make_shared<EEForce>();

    for (auto phase : contact_schedule.GetPhases(ee)) {
      bool   is_contact = phase.first;
      double duration   = phase.second;
      ee_force->AddPhase(duration, dt, is_contact);
    }

    AddComponent(ee_force);         // add to base class for general functionality
    ee_forces_.push_back(ee_force); // keep derived  for specific functionality
  }
}

EndeffectorsForce::~EndeffectorsForce ()
{
}

EndeffectorsForce::LoadParams
EndeffectorsForce::GetLoadValues (double t) const
{
  LoadParams load(ee_ordered_.size());
  for (auto ee : ee_ordered_)
    load.At(ee) = ee_forces_.at(ee)->GetForce(t);

  return load;
}

int
EndeffectorsForce::Index (double t, EndeffectorID ee) const
{
  // get number of previous optimization variables
  int idx_start = 0;
  for (int i=E0; i<ee; ++i)
    idx_start += ee_forces_.at(i)->GetRows();

  // zmp_ highly ugly and error prone...
  // assumes only one optimization variable affects value at time t and endffector e
  return idx_start + ee_forces_.at(ee)->Index(t);
}

} /* namespace opt */
} /* namespace xpp */
