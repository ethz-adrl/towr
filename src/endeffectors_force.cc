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
EndeffectorsForce::GetForce (double t) const
{
  LoadParams load(ee_ordered_.size());
  for (auto ee : ee_ordered_)
    load.At(ee) = ee_forces_.at(ee)->GetForce(t);

  return load;
}

JacobianRow
EndeffectorsForce::GetJacobian (double t, EndeffectorID ee, Coords3D dim) const
{
  JacobianRow jac_row(GetRows());
  JacobianRow jac_ee = ee_forces_.at(ee)->GetJacobian(dim, t);

  int idx_start = 0;
  for (int i=E0; i<ee; ++i)
    idx_start += ee_forces_.at(i)->GetRows();

  // insert single ee-Jacobian into Jacobian representing all endeffectors
  for (JacobianRow::InnerIterator it(jac_ee); it; ++it)
    jac_row.coeffRef(idx_start+it.col()) = it.value();

  return jac_row;
}

} /* namespace opt */
} /* namespace xpp */
