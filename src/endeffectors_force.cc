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


Force::Force (double dt, const ContactSchedule& contact_schedule)
    :Composite("endeffector_load", true)
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

Force::~Force ()
{
}

Force::LoadParams
Force::GetLoadValues (double t) const
{
  LoadParams load(ee_ordered_.size());
  for (auto ee : ee_ordered_)
    load.At(ee) = ee_forces_.at(ee)->GetForce(t);

  return load;
}

int
Force::Index (double t, EndeffectorID ee) const
{
  // get number of previous optimization variables
  int idx_start = 0;
  for (int i=E0; i<ee; ++i)
    idx_start += ee_forces_.at(i)->GetRows();

  // zmp_ highly ugly and error prone...
  // assumes only one optimization variable affects value at time t and endffector e
  return idx_start + ee_forces_.at(ee)->Index(t);
}



/*



EndeffectorsForce::EndeffectorsForce (int num_ee, double dt,
                                  const ContactSchedule& contact_schedule)
    :Component(-1, "endeffector_load")
{
  dt_ = dt;
  T_ = contact_schedule.GetTotalTime();
  n_ee_ = num_ee;
  int idx_segment = GetSegment(T_);
  num_segments_ = idx_segment + 1;
  SetRows(n_ee_*num_segments_);

  double max_load = 2000.0; // [N] limited by robot actuator limits.
  lambdas_ = VectorXd::Ones(GetRows())*max_load/n_ee_;
  SetBounds(contact_schedule, max_load);
}

VecBound
EndeffectorsForce::GetBounds () const
{
  return bounds_;
}

void
EndeffectorsForce::SetBounds (const ContactSchedule& contact_schedule, double max_load)
{
  // just to avoid NaN when for now still calculating CoP from these
  double min_load = 50.0;    // [N] cannot pull on ground (negative forces).

  // sample check if endeffectors are in contact at center of discretization
  // interval.
  for (int k=0; k<num_segments_; ++k) {
    double t = GetTimeCenterSegment(k);
    EndeffectorsBool contacts_state = contact_schedule.IsInContact(t);

    for (bool in_contact : contacts_state.ToImpl()) {
      double lower_bound = in_contact? min_load : 0.0;
      double upper_bound = in_contact? max_load : 0.0;
      bounds_.push_back(Bound(lower_bound, upper_bound));
    }
  }
}

EndeffectorsForce::~EndeffectorsForce ()
{
}

void
EndeffectorsForce::SetValues (const VectorXd& x)
{
  lambdas_ = x;
}

EndeffectorsForce::VectorXd
EndeffectorsForce::GetValues () const
{
  return lambdas_;
}

int
EndeffectorsForce::GetSegment (double t) const
{
  return floor(t/dt_);
}

EndeffectorsForce::LoadParams
EndeffectorsForce::GetLoadValues (double t) const
{
  int k = GetSegment(t);
  return GetLoadValuesIdx(k);
}

EndeffectorsForce::LoadParams
EndeffectorsForce::GetLoadValuesIdx (int k_curr) const
{
  LoadParams load(n_ee_);
  for (auto ee : load.GetEEsOrdered())
    load.At(ee) = lambdas_(IndexDiscrete(k_curr,ee));

  return load;
}

int
EndeffectorsForce::Index (double t, EndeffectorID ee) const
{
  int k = GetSegment(t);
  return IndexDiscrete(k, ee);
}

int
EndeffectorsForce::IndexDiscrete (int k_curr, EndeffectorID ee) const
{
  return n_ee_*k_curr + ee;
}

double
EndeffectorsForce::GetTimeCenterSegment (int segment_id) const
{
  double t_start = segment_id*dt_;

  if (segment_id == num_segments_-1) // last segment might have different length
    return (t_start + T_)/2.;
  else
    return t_start + dt_/2.;
}


*/

} /* namespace opt */
} /* namespace xpp */
