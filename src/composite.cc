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


Primitive::Primitive () : Component(0, "Primitive")
{
}

// make these two different functions
void
Primitive::AddComposite (const OptVarsPtr& vars, int num_rows)
{
  SetRows(num_rows);
  opt_vars_ = vars;
}

Primitive::Jacobian
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


Composite::Composite (const std::string name, bool append_components)
    :Component(0, "C-" + name)
{
  append_components_ = append_components;
  SetName("C-" + name);
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

  assert(false); // component with name doesn't exist
}

Composite::VectorXd
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

Composite::Jacobian
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

} /* namespace opt */
} /* namespace xpp */
