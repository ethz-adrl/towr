/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp/opt/constraints/constraint.h>

namespace xpp {
namespace opt {

int
Constraint::GetNumberOfConstraints () const
{
  return num_constraints_;
}



void
ConstraintLeaf::SetDimensions (const OptVarsPtr& vars,
                           int num_constraints)
{
  num_constraints_ = num_constraints;
  opt_vars_ = vars;
}

ConstraintLeaf::Jacobian
ConstraintLeaf::GetConstraintJacobian () const
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



void
ConstraintComposite::AddConstraint (const ConstraintPtr& constraint)
{
  constraints_.push_back(constraint);
  num_constraints_ += constraint->GetNumberOfConstraints();
}

ConstraintComposite::VectorXd
ConstraintComposite::GetConstraintValues () const
{
  VectorXd g_all(GetNumberOfConstraints());

  int row = 0;
  for (const auto& c : constraints_) {

    VectorXd g = c->GetConstraintValues();
    int n_rows = g.rows();
    g_all.middleRows(row, n_rows) = g;
    row += n_rows;
  }
  return g_all;
}

VecBound
ConstraintComposite::GetBounds () const
{
  VecBound bounds_;
  for (const auto& c : constraints_) {
    VecBound b = c->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }

  return bounds_;
}

ConstraintComposite::Jacobian
ConstraintComposite::GetConstraintJacobian () const
{
  int n_var = constraints_.front()->GetConstraintJacobian().cols();
  Jacobian jacobian(num_constraints_, n_var);

  int row = 0;
  for (const auto& c : constraints_) {

    const Jacobian& jac = c->GetConstraintJacobian();
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(row+it.row(), it.col()) = it.value();

    row += c->GetNumberOfConstraints();
  }

  return jacobian;
}

} /* namespace opt */
} /* namespace xpp */

