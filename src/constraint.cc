/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp/opt/constraints/constraint.h>

namespace xpp {
namespace opt {

Constraint::Constraint ()
{
}

int
Constraint::GetNumberOfConstraints () const
{
  return num_rows_;
}



void
ConstraintLeaf::SetDimensions (const OptVarsPtr& vars, int num_rows)
{
  num_rows_ = num_rows;
  opt_vars_ = vars;
}

ConstraintLeaf::Jacobian
ConstraintLeaf::GetConstraintJacobian () const
{
  Jacobian jacobian(num_rows_, opt_vars_->GetOptimizationVariableCount());

  int col = 0;
  for (const auto& vars : opt_vars_->GetOptVarsVec()) {

    int n = vars->GetOptVarCount();
    Jacobian jac = Jacobian(num_rows_, n);

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
ConstraintComposite::AddConstraint (const BasePtr& constraint)
{
  constraints_.push_back(constraint);

  if (append_components_)
    num_rows_ += constraint->GetNumberOfConstraints();
  else
    num_rows_ = 1; // composite holds costs
}

ConstraintComposite::VectorXd
ConstraintComposite::GetConstraintValues () const
{
  VectorXd g_all = VectorXd::Zero(GetNumberOfConstraints());

  int row = 0;
  for (const auto& c : constraints_) {

    VectorXd g = c->GetConstraintValues();
    int n_rows = g.rows();
    g_all.middleRows(row, n_rows) += g;

    if (append_components_)
      row += n_rows;
  }
  return g_all;
}

ConstraintComposite::Jacobian
ConstraintComposite::GetConstraintJacobian () const
{
  int n_var = constraints_.front()->GetConstraintJacobian().cols();
  Jacobian jacobian(num_rows_, n_var);

  int row = 0;
  for (const auto& c : constraints_) {

    const Jacobian& jac = c->GetConstraintJacobian();
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(row+it.row(), it.col()) += it.value();

    if (append_components_)
      row += c->GetNumberOfConstraints();
  }

  return jacobian;
}

ConstraintComposite::ConstraintComposite (bool append_components)
{
  num_rows_ = 0;
  append_components_ = append_components;
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


} /* namespace opt */
} /* namespace xpp */
