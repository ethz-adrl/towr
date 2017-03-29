/**
 @file    contact_load_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 27, 2017
 @brief   Brief description
 */

#include <xpp/opt/contact_load_constraint.h>

namespace xpp {
namespace opt {

ContactLoadConstraint::ContactLoadConstraint ()
{
  // TODO Auto-generated constructor stub

}

ContactLoadConstraint::~ContactLoadConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
ContactLoadConstraint::Init (const EEMotionPtr& ee_motion,
                             const EELoadPtr& ee_load)
{
  int num_constraints = ee_load->GetOptVarCount();
  SetDependentVariables({ee_motion, ee_load}, num_constraints);

  ee_motion_ = ee_motion;
  ee_load_ = ee_load;

  GetJacobianRefWithRespectTo(ee_load_->GetID()).setIdentity();
}

void
ContactLoadConstraint::UpdateConstraintValues ()
{
  g_ = ee_load_->GetOptimizationParameters();
}

VecBound
ContactLoadConstraint::GetBounds () const
{
  // sample check if beginning and end of motion are not in contact
  // only if both in contact, can lambda be greater than zero
  for (int k=0; k<ee_load_->GetNumberOfSegments(); ++k) {
    double t_start = ee_load_->GetTStart(k);
    double t_end   = ee_load_->GetTStart(k+1) - 1e-5;

    auto contacts_start = ee_motion_->GetContactState(t_start);
    auto contacts_end = ee_motion_->GetContactState(t_end);

    for (auto ee : contacts_start.GetEEsOrdered()) {

      bool contact = contacts_start.At(ee) && contacts_end.At(ee);
      bounds_.at(contacts_start.GetEECount()*k+ee) = Bound(0.0, contact);
    }
  }

  return bounds_;
}

} /* namespace opt */
} /* namespace xpp */
