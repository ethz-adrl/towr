/**
 @file    nlp.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <opt_solve/nlp.h>


namespace opt {

NLP::NLP ()
{
}

void
NLP::SetVariables (const Component::Ptr& opt_variables)
{
  opt_variables_ = opt_variables;
  x_prev.clear();
}

int
NLP::GetNumberOfOptimizationVariables () const
{
  return opt_variables_->GetRows();
}

NLP::VecBound
NLP::GetBoundsOnOptimizationVariables () const
{
  return opt_variables_->GetBounds();
}

NLP::VectorXd
NLP::GetStartingValues () const
{
  return opt_variables_->GetValues();
}

void
NLP::SetVariables (const double* x)
{
  opt_variables_->SetValues(ConvertToEigen(x));
}

double
NLP::EvaluateCostFunction (const double* x)
{
  VectorXd g = VectorXd::Zero(1);
  if (HasCostTerms()) {
    SetVariables(x);
    g = costs_->GetValues();
  }
  return g(0);
}

NLP::VectorXd
NLP::EvaluateCostFunctionGradient (const double* x)
{
  Jacobian jac = Jacobian(1,GetNumberOfOptimizationVariables());
  if (HasCostTerms()) {
    SetVariables(x);
    jac = costs_->GetJacobian();
  }

  return jac.row(0).transpose();
}

NLP::VecBound
NLP::GetBoundsOnConstraints () const
{
  return constraints_->GetBounds();
}

int
NLP::GetNumberOfConstraints () const
{
  return GetBoundsOnConstraints().size();
}

NLP::VectorXd
NLP::EvaluateConstraints (const double* x)
{
  SetVariables(x);
  return constraints_->GetValues();
}

bool
NLP::HasCostTerms () const
{
  return costs_->GetRows()>0;
}

void
NLP::EvalNonzerosOfJacobian (const double* x, double* values)
{
  SetVariables(x);
  Jacobian jac = GetJacobianOfConstraints();

  jac.makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

NLP::Jacobian
NLP::GetJacobianOfConstraints () const
{
  return constraints_->GetJacobian();
}

void
NLP::SetCosts (Component::PtrU cost)
{
  costs_ = std::move(cost);
}

void
NLP::SetConstraints (Component::PtrU constraint)
{
  constraints_ = std::move(constraint);
}

void
NLP::PrintCurrent() const
{
  opt_variables_->Print();
  costs_->Print();
  constraints_->Print();
};

void
NLP::SaveCurrent()
{
  x_prev.push_back(opt_variables_->GetValues());
}

Component::Ptr
NLP::GetOptVariables ()
{
  return GetOptVariables(GetIterationCount()-1);
}

Component::Ptr
NLP::GetOptVariables (int iter)
{
  opt_variables_->SetValues(x_prev.at(iter));
  return opt_variables_;
}

NLP::VectorXd
NLP::ConvertToEigen(const double* x) const
{
  return Eigen::Map<const VectorXd>(x,GetNumberOfOptimizationVariables());
}

NLP::~NLP ()
{
}

} /* namespace opt */

