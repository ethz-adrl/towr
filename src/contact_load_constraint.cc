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

  // sample check if endeffectors are in contact at center of discretization
  // inverval.
  for (int segment=0; segment<ee_load_->GetNumberOfSegments(); ++segment) {
    double t_center = ee_load_->GetTimeCenterSegment(segment);
    auto contacts_center = ee_motion_->GetContactState(t_center);

    for (auto ee : contacts_center.GetEEsOrdered()) {
      auto contact = static_cast<double>(contacts_center.At(ee));
      bounds_.at(contacts_center.GetEECount()*segment+ee) = Bound(0.0, contact);
    }
  }
}

} /* namespace opt */
} /* namespace xpp */
