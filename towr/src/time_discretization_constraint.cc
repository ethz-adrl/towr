/**
 @file    time_discretization_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 30, 2017
 @brief   Brief description
 */

#include <towr/constraints/time_discretization_constraint.h>

#include <cmath>
#include <initializer_list>
#include <Eigen/Dense>

namespace xpp {

using namespace opt;

TimeDiscretizationConstraint::TimeDiscretizationConstraint (double T, double dt,
                                                            const std::string& name)
    :ConstraintSet(kSpecifyLater, name)
{
  double t = 0.0;
  dts_ = {t};

  for (int i=0; i<floor(T/dt); ++i) {
    t += dt;
    dts_.push_back(t);
  }

  dts_.push_back(T); // also ensure constraints at very last node/time.
//  AddOptimizationVariables(opt_vars);
}

TimeDiscretizationConstraint::TimeDiscretizationConstraint (const EvaluationTimes& times,
                                                            const std::string& name)
   :ConstraintSet(kSpecifyLater, name) // just placeholder
{
  dts_ = times;
//  AddOptimizationVariables(opt_vars);
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
TimeDiscretizationConstraint::GetValues () const
{
  VectorXd g = VectorXd::Zero(GetRows());

  int k = 0;
  for (double t : dts_)
    UpdateConstraintAtInstance(t, k++, g);

  return g;
}

TimeDiscretizationConstraint::VecBound
TimeDiscretizationConstraint::GetBounds () const
{
  VecBound bounds(GetRows());

  int k = 0;
  for (double t : dts_)
    UpdateBoundsAtInstance(t, k++, bounds);

  return bounds;
}

void
TimeDiscretizationConstraint::FillJacobianBlock (std::string var_set,
                                                  Jacobian& jac) const
{
  int k = 0;
  for (double t : dts_)
    UpdateJacobianAtInstance(t, k++, jac, var_set);
}

} /* namespace xpp */


