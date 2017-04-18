/**
 @file    time_discretization_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 30, 2017
 @brief   Brief description
 */

#include <xpp/opt/constraints/time_discretization_constraint.h>
#include <cmath>

namespace xpp {
namespace opt {

TimeDiscretizationConstraint::TimeDiscretizationConstraint (double T, double dt,
                                                            int constraints_per_time)
{
  dts_.clear();
  double t = 0.0;
  for (int i=0; i<floor(T/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }

  int num_constraints = GetNumberOfNodes()*constraints_per_time;
  g_new_  = VectorXd(num_constraints);
  bounds_ = VecBound(num_constraints);

}

TimeDiscretizationConstraint::~TimeDiscretizationConstraint ()
{
}

int
TimeDiscretizationConstraint::GetNumberOfNodes () const
{
  return dts_.size();
}

TimeDiscretizationConstraint::VectorXd
TimeDiscretizationConstraint::GetConstraintValues () const
{
  int k = 0;
  for (double t : dts_)
    UpdateConstraintAtInstance(t, k++);

  return g_new_;
}

VecBound
TimeDiscretizationConstraint::GetBounds () const
{
  int k = 0;
  for (double t : dts_)
    UpdateBoundsAtInstance(t, k++);

  return bounds_;
}

void
TimeDiscretizationConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                        Jacobian& jac) const
{
  int k = 0;
  for (double t : dts_)
    UpdateJacobianAtInstance(t, k++, jac, var_set);
}

//void
//TimeDiscretizationConstraint::UpdateJacobians ()
//{
//  // zmp_ remove
////  int k = 0;
////  for (double t : dts_)
//////    UpdateJacobianAtInstance(t, k++);
//}

} /* namespace opt */
} /* namespace xpp */


