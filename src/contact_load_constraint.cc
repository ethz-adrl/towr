/**
 @file    contact_load_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 27, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/contact_load_constraint.h>

namespace xpp {
namespace opt {

ContactLoadConstraint::ContactLoadConstraint (const EEMotionPtr& ee_motion,
                                              const EELoadPtr& ee_load)
{

  int num_constraints = ee_load->GetOptVarCount();
  SetDependentVariables({ee_motion, ee_load}, num_constraints);

  ee_motion_ = ee_motion;
  ee_load_ = ee_load;

  GetJacobianRefWithRespectTo(ee_load_->GetID()).setIdentity();
}

ContactLoadConstraint::~ContactLoadConstraint ()
{
}

void
ContactLoadConstraint::UpdateConstraintValues ()
{
  g_ = ee_load_->GetOptimizationParameters();
}

void
ContactLoadConstraint::UpdateBounds ()
{
  // spring_clean_ the load discretization should be more tightly
  // coupled to the endeffector motion

  // sample check if beginning and end of motion are not in contact
  // only if both in contact, can lambda be greater than zero
  for (int segment=0; segment<ee_load_->GetNumberOfSegments(); ++segment) {
    double t_start = ee_load_->GetTStart(segment);
    double t_end   = ee_load_->GetTEnd(segment)-1e-5;

    auto contacts_start = ee_motion_->GetContactState(t_start);
    auto contacts_end   = ee_motion_->GetContactState(t_end);

    for (auto ee : contacts_start.GetEEsOrdered()) {

      bool contact = contacts_start.At(ee) && contacts_end.At(ee);
      bounds_.at(contacts_start.GetEECount()*segment+ee) = Bound(0.0, contact);
    }
  }
}

} /* namespace opt */
} /* namespace xpp */
