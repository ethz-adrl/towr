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

#include <opt_solve/problem.h>


namespace opt {

Problem::Problem ()
{
}

void
Problem::SetVariables (const Component::Ptr& opt_variables)
{
  opt_variables_ = opt_variables;
  x_prev.clear();
}

int
Problem::GetNumberOfOptimizationVariables () const
{
  return opt_variables_->GetRows();
}

Problem::VecBound
Problem::GetBoundsOnOptimizationVariables () const
{
  return opt_variables_->GetBounds();
}

Problem::VectorXd
Problem::GetStartingValues () const
{
  return opt_variables_->GetValues();
}

void
Problem::SetVariables (const double* x)
{
  opt_variables_->SetVariables(ConvertToEigen(x));
}

double
Problem::EvaluateCostFunction (const double* x)
{
  VectorXd g = VectorXd::Zero(1);
  if (HasCostTerms()) {
    SetVariables(x);
    g = costs_->GetValues();
  }
  return g(0);
}

Problem::VectorXd
Problem::EvaluateCostFunctionGradient (const double* x)
{
  Jacobian jac = Jacobian(1,GetNumberOfOptimizationVariables());
  if (HasCostTerms()) {
    SetVariables(x);
    jac = costs_->GetJacobian();
  }

  return jac.row(0).transpose();
}

Problem::VecBound
Problem::GetBoundsOnConstraints () const
{
  return constraints_->GetBounds();
}

int
Problem::GetNumberOfConstraints () const
{
  return GetBoundsOnConstraints().size();
}

Problem::VectorXd
Problem::EvaluateConstraints (const double* x)
{
  SetVariables(x);
  return constraints_->GetValues();
}

bool
Problem::HasCostTerms () const
{
  return costs_->GetRows()>0;
}

void
Problem::EvalNonzerosOfJacobian (const double* x, double* values)
{
  SetVariables(x);
  Jacobian jac = GetJacobianOfConstraints();

  jac.makeCompressed(); // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

Problem::Jacobian
Problem::GetJacobianOfConstraints () const
{
  return constraints_->GetJacobian();
}

void
Problem::SetCosts (Component::PtrU cost)
{
  costs_ = std::move(cost);
}

void
Problem::SetConstraints (Component::PtrU constraint)
{
  constraints_ = std::move(constraint);
}

void
Problem::PrintCurrent() const
{
  opt_variables_->Print();
  costs_->Print();
  constraints_->Print();
};

void
Problem::SaveCurrent()
{
  x_prev.push_back(opt_variables_->GetValues());
}

Component::Ptr
Problem::GetOptVariables ()
{
  return GetOptVariables(GetIterationCount()-1);
}

Component::Ptr
Problem::GetOptVariables (int iter)
{
  opt_variables_->SetVariables(x_prev.at(iter));
  return opt_variables_;
}

Problem::VectorXd
Problem::ConvertToEigen(const double* x) const
{
  return Eigen::Map<const VectorXd>(x,GetNumberOfOptimizationVariables());
}

Problem::~Problem ()
{
}

} /* namespace opt */

