/**
 @file    contact_load_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 27, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/depr/contact_load_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/bound.h>
#include <xpp/opt/variables/contact_schedule.h>
#include <xpp/opt/variables/endeffector_load.h>

namespace xpp {
namespace opt {

ContactLoadConstraint::ContactLoadConstraint (const OptVarsPtr& opt_vars)
{
  name_ = "ContactLoadConstraint";
  contact_schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetSet("contact_schedule"));
  ee_load_          = std::dynamic_pointer_cast<EndeffectorLoad>(opt_vars->GetSet("endeffector_load"));

  int num_constraints = ee_load_->GetOptVarCount();
  SetDimensions(opt_vars, num_constraints);

  ee_ids_  = contact_schedule_->IsInContact(0.0).GetEEsOrdered();
}

ContactLoadConstraint::~ContactLoadConstraint ()
{
}

ContactLoadConstraint::VectorXd
ContactLoadConstraint::GetValues () const
{
  return ee_load_->GetVariables();
}

VecBound
ContactLoadConstraint::GetBounds () const
{
  VecBound bounds;
  double max_load = 1000.0; // [N] limited by robot actuator limits.
  double min_load = 0.0;    // [N] cannot pull on ground (negative forces).

  // sample check if endeffectors are in contact at center of discretization
  // interval.
  for (int k=0; k<ee_load_->GetNumberOfSegments(); ++k) {
    double t_center = ee_load_->GetTimeCenterSegment(k);
    EndeffectorsBool contacts_center = contact_schedule_->IsInContact(t_center);

    for (auto ee : ee_ids_) {
      double bound = contacts_center.At(ee)? max_load : 0.0;
      bounds.push_back(Bound(min_load, bound));
    }
  }

  return bounds;
}

void
ContactLoadConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                 Jacobian& jac) const
{
  if (var_set == ee_load_->GetId()) {
    int row = 0;
    for (int k=0; k<ee_load_->GetNumberOfSegments(); ++k) {
      for (auto ee : ee_ids_) {
        int idx = ee_load_->IndexDiscrete(k,ee);
        jac.insert(row++, idx) = 1.0;
      }
    }
  }
}

} /* namespace opt */
} /* namespace xpp */


