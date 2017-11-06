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
 @file    nlp.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#ifndef OPT_SOLVE_INCLUDE_OPT_NLP_H_
#define OPT_SOLVE_INCLUDE_OPT_NLP_H_


#include "composite.h"

namespace opt {

/** @brief Nonlinear Programming problem definition
  *
  * This class is responsible for holding all the information of a
  * Nonlinear Program, which includes the optimization variables, their bounds,
  * the cost function, the constraint function, constraint bounds and possibly
  * derivatives.
  */
class NLP {
public:
  using VecBound = Component::VecBound;
  using Jacobian = Component::Jacobian;
  using VectorXd = Component::VectorXd;


  NLP ();
  virtual ~NLP ();

  void SetVariables(const Component::Ptr& variables);
  void SetCosts(Component::PtrU);
  void SetConstraints(Component::PtrU);



  void SetVariables(const double* x);

  int GetNumberOfOptimizationVariables() const;
  bool HasCostTerms() const;
  VecBound GetBoundsOnOptimizationVariables() const;
  VectorXd GetStartingValues() const;

  double EvaluateCostFunction(const double* x);
  VectorXd EvaluateCostFunctionGradient(const double* x);

  int GetNumberOfConstraints() const;
  VecBound GetBoundsOnConstraints() const;
  VectorXd EvaluateConstraints(const double* x);

  void EvalNonzerosOfJacobian(const double* x, double* values);
  Jacobian GetJacobianOfConstraints() const;


  void PrintCurrent() const;

  /** @brief saves the current values of the optimization variables.
   *
   *  This is used to keep a history of the values for each NLP iterations.
   */
  void SaveCurrent();

  Component::Ptr GetOptVariables();
  Component::Ptr GetOptVariables(int iter);
  int GetIterationCount() const { return x_prev.size(); };

private:
  Component::PtrU constraints_;
  Component::PtrU costs_;
  Component::Ptr opt_variables_;

  std::vector<VectorXd> x_prev;

  VectorXd ConvertToEigen(const double* x) const;
};

} /* namespace opt */

#endif /* OPT_SOLVE_INCLUDE_OPT_NLP_H_ */
