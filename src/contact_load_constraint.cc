/**
 @file    contact_load_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 27, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/contact_load_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/bound.h>
#include <xpp/opt/variables/contact_schedule.h>
#include <xpp/opt/variables/endeffector_load.h>

namespace xpp {
namespace opt {

ContactLoadConstraint::ContactLoadConstraint (const OptVarsPtr& opt_vars)
{
  contact_schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetSet("contact_schedule"));
  ee_load_          = std::dynamic_pointer_cast<EndeffectorLoad>(opt_vars->GetSet("endeffector_load"));

  int num_constraints = ee_load_->GetOptVarCount();
  SetDimensions(opt_vars->GetOptVarsVec(), num_constraints);

  ee_ids_  = contact_schedule_->IsInContact(0.0).GetEEsOrdered();

  GetJacobianRefWithRespectTo(ee_load_->GetId()).setIdentity();
}

ContactLoadConstraint::~ContactLoadConstraint ()
{
}

ContactLoadConstraint::VectorXd
ContactLoadConstraint::GetConstraintValues () const
{
  return ee_load_->GetVariables();
}

void
ContactLoadConstraint::UpdateBounds ()
{
  // spring_clean_ the load discretization should be more tightly
  // coupled to the endeffector motion

  // sample check if endeffectors are in contact at center of discretization
  // inverval.
  for (int segment=0; segment<ee_load_->GetNumberOfSegments(); ++segment) {
    double t_center = ee_load_->GetTimeCenterSegment(segment);
    EndeffectorsBool contacts_center = contact_schedule_->IsInContact(t_center);

    for (auto ee : ee_ids_) {
      double bound = contacts_center.At(ee)? 1.0 : 0.0;
      bounds_.at(ee_ids_.size()*segment+ee) = Bound(0.0, bound);
    }
  }
}

} /* namespace opt */
} /* namespace xpp */


