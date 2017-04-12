/**
 @file    time_discretization_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 30, 2017
 @brief   Brief description
 */

#include <xpp/time_discretization_constraint.h>
#include <cmath>

namespace xpp {
namespace opt {

TimeDiscretizationConstraint::TimeDiscretizationConstraint (double T, double dt)
{
  dts_.clear();
  double t = 0.0;
  for (int i=0; i<floor(T/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }
}

TimeDiscretizationConstraint::~TimeDiscretizationConstraint ()
{
}

int
TimeDiscretizationConstraint::GetNumberOfNodes () const
{
  return dts_.size();
}

void
TimeDiscretizationConstraint::UpdateConstraintValues ()
{
  int k = 0;
  for (double t : dts_)
    UpdateConstraintAtInstance(t, k++);
}

void
TimeDiscretizationConstraint::UpdateBounds ()
{
  int k = 0;
  for (double t : dts_)
    UpdateBoundsAtInstance(t, k++);
}

void
TimeDiscretizationConstraint::UpdateJacobians ()
{
  int k = 0;
  for (double t : dts_)
    UpdateJacobianAtInstance(t, k++);
}

} /* namespace opt */
} /* namespace xpp */
