/**
 @file    cost_function_functor.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 3, 2016
 @brief   Defines a class to calculate numerical derivatives through Eigen
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_FUNCTION_FUNCTOR_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_FUNCTION_FUNCTOR_H_

#include "eigen_num_diff_functor.h"
#include "cost_container.h"
#include "optimization_variables.h"

namespace xpp {
namespace opt {

/** @brief Calculates the derivatives of the cost function.
  *
  * This class is responsible for supplying the value of the cost function
  * evaluated for the current value of the optimization variables. This
  * operator() is then used with Eigen/NumericalDiff to calculate the derivatives.
  */
class CostFunctionFunctor : public utils::EigenNumDiffFunctor<double> {
public:
  typedef utils::EigenNumDiffFunctor<double> Base;

  CostFunctionFunctor ();
  virtual ~CostFunctionFunctor ();

  void AddCosts(OptimizationVariables& subject, CostContainer& costs);

protected:
  int operator() (const InputType& x, ValueType& obj_value) const override;

private:
  OptimizationVariables* subject_;
  CostContainer* cost_container_;

  bool costs_added_ = false; // make sure this functor has been assigned a cost function
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_COST_FUNCTION_FUNCTOR_H_ */
