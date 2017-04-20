/**
 @file    composite.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

Component::Component ()
{
}

int
Component::GetRows () const
{
  return num_rows_;
}

void
Component::Print () const
{
  std::cout << num_rows_ << "\t(" << name_ << ")" << std::endl;
}


void
Primitive::SetDimensions (const OptVarsPtr& vars, int num_rows)
{
  num_rows_ = num_rows;
  opt_vars_ = vars;
}

Primitive::Jacobian
Primitive::GetJacobian () const
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


Composite::Composite (const std::string name, bool append_components)
{
  num_rows_ = 0; // "meat" can only be added by primitive components
  append_components_ = append_components;
  name_ = "C-" + name;
}


void
Composite::AddComponent (const ComponentPtr& constraint)
{
  components_.push_back(constraint);

  if (append_components_)
    num_rows_ += constraint->GetRows();
  else
    num_rows_ = 1; // composite holds costs
}

Composite::VectorXd
Composite::GetValues () const
{
  VectorXd g_all = VectorXd::Zero(GetRows());

  int row = 0;
  for (const auto& c : components_) {

    VectorXd g = c->GetValues();
    int n_rows = g.rows();
    g_all.middleRows(row, n_rows) += g;

    if (append_components_)
      row += n_rows;
  }
  return g_all;
}

Composite::Jacobian
Composite::GetJacobian () const
{
  int n_var = components_.front()->GetJacobian().cols();
  Jacobian jacobian(num_rows_, n_var);

  int row = 0;
  for (const auto& c : components_) {

    const Jacobian& jac = c->GetJacobian();
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(row+it.row(), it.col()) += it.value();

    if (append_components_)
      row += c->GetRows();
  }

  return jacobian;
}

VecBound
Composite::GetBounds () const
{
  VecBound bounds_;
  for (const auto& c : components_) {
    VecBound b = c->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }

  return bounds_;
}

void
Composite::Print () const
{
  std::cout << name_ << ":\n";
  for (auto c : components_) {
    std::cout << "    "; // indent components
    c->Print();
  }
  std::cout << std::endl;
}

} /* namespace opt */
} /* namespace xpp */
