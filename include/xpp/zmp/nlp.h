/**
 @file    nlp.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_H_

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/constraint_container.h>
#include <xpp/zmp/cost_container.h>
#include <xpp/zmp/cost_function_functor.h>

#include <memory>

namespace xpp {
namespace zmp {

/** @brief Nonlinear Programming problem definition
  *
  * This class is responsible for holding all the information of a
  * Nonlinear Program, which includes the optimization variables, their bounds,
  * the cost function, the constraint function, constraint bounds  and possibly
  * derivatives.
  */
class NLP {
public:
  typedef AConstraint::Bound Bound;
  typedef AConstraint::Jacobian Jacobian;
  typedef double Number;
  typedef Eigen::VectorXd VectorXd;
  typedef std::vector<Bound> BoundVec;
  typedef Eigen::NumericalDiff<CostFunctionFunctor> NumericalDiffFunctor;


  typedef std::shared_ptr<OptimizationVariables> OptimizationVariablesPtr;
  typedef std::shared_ptr<CostContainer> CostContainerPtr;
  typedef std::shared_ptr<ConstraintContainer> ConstraintContainerPtr;

  NLP ();
  virtual ~NLP ();

  void Init(OptimizationVariablesPtr&, CostContainerPtr&, ConstraintContainerPtr&);
  void SetVariables(const Number* x);

  int GetNumberOfOptimizationVariables() const;
  bool HasCostTerms() const;
  BoundVec GetBoundsOnOptimizationVariables() const;
  VectorXd GetStartingValues() const;

  double EvaluateCostFunction(const Number* x) const;
  VectorXd EvaluateCostFunctionGradient(const Number* x) const;

  int GetNumberOfConstraints() const;
  BoundVec GetBoundsOnConstraints() const;
  VectorXd EvaluateConstraints(const Number* x) const;

  void EvalNonzerosOfJacobian(const Number* x, Number* values) const;
  Jacobian GetJacobianOfConstraints() const;

private:
  OptimizationVariablesPtr opt_variables_;
  CostContainerPtr costs_;
  ConstraintContainerPtr constraints_;

  NumericalDiffFunctor cost_derivative_;

  VectorXd ConvertToEigen(const Number* x) const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_H_ */
