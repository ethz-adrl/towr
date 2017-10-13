/**
 @file    contact_constraints.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 7, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/contact_constraints.h>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/variables/contact_timings.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

ContactConstraints::ContactConstraints (const OptVarsPtr& opt_vars,
                                        double T, double dt,
                                        EndeffectorID ee)
    :TimeDiscretizationConstraint(T, dt, opt_vars)
{
  SetName("ContactConstraints-" + std::to_string(ee));

  ee_motion_       = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::endeffectors_motion+std::to_string(ee)));
  contact_timings_ = std::dynamic_pointer_cast<ContactTimings>  (opt_vars->GetComponent(id::contact_timings    +std::to_string(ee)));

  // constraining the velocity in xyz
  int num_constraints = GetNumberOfNodes()*constraints_per_node;
  SetRows(num_constraints);
}

ContactConstraints::~ContactConstraints ()
{
  // TODO Auto-generated destructor stub
}

int
ContactConstraints::GetRow (int node, int c) const
{
  return node*constraints_per_node + c;
}

void
ContactConstraints::UpdateConstraintAtInstance (double t, int k,
                                                VectorXd& g) const
{
  Vector3d c = contact_timings_->GetContactValue(t)*Vector3d::Ones();
  Vector3d xd = ee_motion_->GetPoint(t).v_;
  g.middleRows(GetRow(k, 0), kDim3d) = 1.0/max_vel_*xd + c; // complementary constraint, is this hard?
  g.middleRows(GetRow(k, 3), kDim3d) = 1.0/max_vel_*xd - c; // complementary constraint, is this hard?
}

void
ContactConstraints::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  int i=0;
  for (int dim=0; dim<kDim3d; ++dim) {
    bounds.at(GetRow(k,i++)) = Bound(-1.0e20, 1.0); // possibly relax to help finding solutions
  }

  for (int dim=0; dim<kDim3d; ++dim) {
    bounds.at(GetRow(k,i++)) = Bound(-1.0, 1.0e20); // possibly relax to help finding solutions
  }
}

void
ContactConstraints::UpdateJacobianAtInstance (double t, int k,
                                              Jacobian& jac,
                                              std::string var_set) const
{
  int row_start = GetRow(k,X);

  if (var_set == ee_motion_->GetName()) {
    double c = contact_timings_->GetContactValue(t);
    jac.middleRows(row_start, kDim3d)   = 1.0/max_vel_*c*ee_motion_->GetJacobian(t,kVel);
    jac.middleRows(row_start+3, kDim3d) = 1.0/max_vel_*c*ee_motion_->GetJacobian(t,kVel);
  }

  if (var_set == contact_timings_->GetName()) {
    int i=row_start;
    Vector3d xd = ee_motion_->GetPoint(t).v_;
    for (int dim=0; dim<kDim3d; ++dim) {
      jac.row(i++) = contact_timings_->GetJacobianOfContactValueWrtTimings(t);
    }

    for (int dim=0; dim<kDim3d; ++dim) {
      jac.row(i++) = -contact_timings_->GetJacobianOfContactValueWrtTimings(t);
    }
  }

}

} /* namespace opt */
} /* namespace xpp */
