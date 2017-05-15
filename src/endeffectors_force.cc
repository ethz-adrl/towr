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

EndeffectorsForce::EndeffectorsForce (int num_ee, double dt, double T,
                                  const ContactSchedule& contact_schedule)
    :Component(-1, "endeffector_load")
{
  dt_ = dt;
  T_ = T;
  n_ee_ = num_ee;
  int idx_segment = GetSegment(T);
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

//int
//EndeffectorLoad::GetNumberOfSegments () const
//{
//  return num_segments_;
//}

} /* namespace opt */
} /* namespace xpp */
