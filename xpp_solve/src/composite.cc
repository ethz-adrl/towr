/**
 @file    composite.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#include <xpp_solve/composite.h>

#include <iostream>

namespace xpp {

static int print_counter = 0;

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

std::string
Component::GetName () const
{
  return name_;
}


Composite::Composite (const std::string name, bool is_cost)
    :Component(0, name)
{
  is_cost_ = is_cost;
}

void
Composite::AddComponent (const Component::Ptr& c)
{
  components_.push_back(c);

  if (is_cost_)
    SetRows(1);
  else
    SetRows(GetRows()+ c->GetRows());
}

void
Composite::ClearComponents ()
{
  components_.clear();
  SetRows(0);
}

Component::Ptr
Composite::GetComponent (std::string name) const
{
  for (const auto& c : components_)
    if (c->GetName() == name)
      return c;

  assert(false); // component with name doesn't exist
}

VectorXd
Composite::GetValues () const
{
  VectorXd g_all = VectorXd::Zero(GetRows());

  int row = 0;
  for (const auto& c : GetNonzeroComponents()) {

    int n_rows = c->GetRows();
    VectorXd g = c->GetValues();
    g_all.middleRows(row, n_rows) += g;

    if (!is_cost_)
      row += n_rows;
  }
  return g_all;
}

void
Composite::SetValues (const VectorXd& x)
{
  int row = 0;
  for (auto& c : GetNonzeroComponents()) {

    int n_rows = c->GetRows();
    c->SetValues(x.middleRows(row,n_rows));
    row += n_rows;
  }
}

Jacobian
Composite::GetJacobian () const
{
  int n_var = GetNonzeroComponents().front()->GetJacobian().cols();
  Jacobian jacobian(GetRows(), n_var);

  int row = 0;
  for (const auto& c : GetNonzeroComponents()) {

    const Jacobian& jac = c->GetJacobian();
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(row+it.row(), it.col()) += it.value();

    if (!is_cost_)
      row += c->GetRows();
  }

  return jacobian;
}

VecBound
Composite::GetBounds () const
{
  VecBound bounds_;
  for (const auto& c : GetNonzeroComponents()) {
    VecBound b = c->GetBounds();
    bounds_.insert(bounds_.end(), b.begin(), b.end());
  }

  return bounds_;
}

Composite::ComponentVec
Composite::GetNonzeroComponents() const
{
  ComponentVec components;
  for (const auto& c : components_)
    if (c->GetRows() != 0 )
      components.push_back(c);

  return components;
}

// some printouts for convenience
void
Component::Print () const
{
  int print_rows = 3;
  std::string end_string = ", ...";

  if (num_rows_ < print_rows) {
    print_rows = num_rows_;
    end_string.clear(); // all variables printed
  }

  // calculate squared bound violation
  VectorXd x = GetValues();
  VecBound bounds = GetBounds();

  std::vector<int> viol_idx;
  double eps = 0.001; // from ipopt config file
  for (int i=0; i<bounds.size(); ++i) {
    double lower = bounds.at(i).lower_;
    double upper = bounds.at(i).upper_;
    double val = x(i);
    if (val < lower-eps || upper+eps < val)
      viol_idx.push_back(i); // constraint out of bounds
  }


  std::cout.precision(2);
  std::cout << std::fixed;
  // https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
  std::string black = "\033[0m";
  std::string red   = "\033[31m";
  std::string color = viol_idx.empty()? black : red;
  std::cout << name_ << "\t(";
  std::cout << num_rows_ << ", " << print_counter << "-" << print_counter+num_rows_;
  std::cout << ", " << color << "nr_violated=" << viol_idx.size() << " ( ";
  int i_print = 4;
  int nr_indices_print = viol_idx.size()<i_print? viol_idx.size() : i_print;
  for (int i=0; i<nr_indices_print; ++i)
    std::cout << viol_idx.at(i) << ", ";
  std::cout << ")";
  std::cout << black;
  std::cout << ":\t";

  print_counter += num_rows_;

  VectorXd val = GetValues().topRows(print_rows);
  if (val.rows() > 0)
    std::cout << val(0);
  for (int i=1; i<val.rows(); ++i)
    std::cout << ",\t" << val(i);

  std::cout << end_string << std::endl;
}


void
Composite::Print () const
{
  if (GetName()=="nlp_variables" || GetName()=="constraints")
    print_counter = 0;

  std::cout << GetName() << ":\n";
  for (auto c : GetNonzeroComponents()) {
    std::cout << "   "; // indent components
    c->Print();
  }
  std::cout << std::endl;
}



} /* namespace xpp */

