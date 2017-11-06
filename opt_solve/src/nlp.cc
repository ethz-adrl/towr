/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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

