/**
 @file    composite.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

Component::Component (int num_rows, const std::string name)
{
  num_rows_ = num_rows;
  name_ = name;
}

int
Component::GetRows () const
{
  return num_rows_;
}

void
Component::SetRows (int num_rows)
{
  num_rows_ = num_rows;
}

void
Component::Print () const
{
  std::cout << num_rows_ << "\t(" << name_ << ")" << std::endl;
}

std::string
Component::GetName () const
{
  return name_;
}

void
Component::SetName (const std::string& name)
{
  name_ = name;
}

Composite::Composite (const std::string name, bool append_components)
    :Component(0, name)
{
  append_components_ = append_components;
}

void
Composite::AddComponent (const ComponentPtr& c)
{
  components_.push_back(c);

  if (append_components_)
    SetRows(GetRows()+ c->GetRows());
  else
    SetRows(1); // composite holds costs
}

void
Composite::ClearComponents ()
{
  components_.clear();
  SetRows(0);
}

Composite::ComponentVec
Composite::GetComponents () const
{
  return components_;
}

Composite::ComponentPtr
Composite::GetComponent (std::string name) const
{
  for (const auto& c : components_)
    if (c->GetName() == name)
      return c;

  std::cerr << "component \"" << name << "\" doesn't exist." << std::endl;
  assert(false); // component with name doesn't exist
}

VectorXd
Composite::GetValues () const
{
  VectorXd g_all = VectorXd::Zero(GetRows());

  int row = 0;
  for (const auto& c : components_) {

    VectorXd g = c->GetValues();
    int n_rows = c->GetRows();
    g_all.middleRows(row, n_rows) += g;

    if (append_components_)
      row += n_rows;
  }
  return g_all;
}

void
Composite::SetValues (const VectorXd& x)
{
  int row = 0;
  for (auto& c : components_) {
    int n_var = c->GetRows();
    c->SetValues(x.middleRows(row,n_var));
    row += n_var;
  }
}

Jacobian
Composite::GetJacobian () const
{
  int n_var = components_.front()->GetJacobian().cols();
  Jacobian jacobian(GetRows(), n_var);

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

int
Composite::GetComponentCount () const
{
  return components_.size();
}

void
Composite::Print () const
{
  std::cout << GetName() << ":\n";
  for (auto c : components_) {
    std::cout << "    "; // indent components
    c->Print();
  }
  std::cout << std::endl;
}

Primitive::Primitive () : Component(-1, "Primitive")
{
}

void
Primitive::AddComposite (const OptVarsPtr& vars)
{
  opt_vars_ = vars;
}

Jacobian
Primitive::GetJacobian () const
{
  Jacobian jacobian(GetRows(), opt_vars_->GetRows());

  int col = 0;
  for (const auto& vars : opt_vars_->GetComponents()) {

    int n = vars->GetRows();
    Jacobian jac = Jacobian(GetRows(), n);

    FillJacobianWithRespectTo(vars->GetName(), jac);

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
