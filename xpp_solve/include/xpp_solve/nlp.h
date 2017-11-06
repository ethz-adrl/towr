/**
 @file    nlp.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_NLP_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_NLP_H_


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

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_NLP_H_ */
