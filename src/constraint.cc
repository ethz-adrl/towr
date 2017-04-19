/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp/opt/constraints/constraint.h>

namespace xpp {
namespace opt {

ConstraintBase::ConstraintBase ()
{
}

ConstraintBase::~ConstraintBase ()
{
}

int
ConstraintBase::GetNumberOfConstraints () const
{
  return num_constraints_;
}

// zmp_ remove
//int
//Constraint::GetNumberOfOptVariables () const
//{
//  return opt_vars_->GetOptimizationVariableCount();
//}

void
Constraint::SetDimensions (const OptVarsPtr& vars,
                           int num_constraints)
{
  num_constraints_ = num_constraints;
  opt_vars_ = vars;
}

Constraint::Jacobian
Constraint::GetConstraintJacobian () const
{
  Jacobian jacobian(num_constraints_, opt_vars_->GetOptimizationVariableCount());

  int col = 0;
  for (const auto& vars : opt_vars_->GetOptVarsVec()) {

    int n = vars->GetOptVarCount();
    Jacobian jac = Jacobian(num_constraints_, n);

    FillJacobianWithRespectTo(vars->GetId(), jac);

    // insert the derivative in the correct position in the overall Jacobian
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(it.row(), col+it.col()) = it.value();

    col += n;
  }

  return jacobian;
}

} /* namespace opt */
} /* namespace xpp */

