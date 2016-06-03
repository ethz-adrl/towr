/**
 @file    cost_function_functor.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 3, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION_FUNCTOR_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION_FUNCTOR_H_

#include <xpp/utils/eigen_num_diff_functor.h>
#include <xpp/zmp/cost_container.h>
#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

class CostFunctionFunctor : public utils::EigenNumDiffFunctor<double> {
public:
  typedef utils::EigenNumDiffFunctor<double> Base;

  CostFunctionFunctor ();
  virtual ~CostFunctionFunctor ();

  void AddCosts(OptimizationVariables& subject, CostContainer& costs);

  int operator() (const InputType& x, ValueType& obj_value) const override;

private:
  OptimizationVariables* subject_;
  CostContainer* cost_container_;

  bool costs_added_ = false; // make sure this functor has been assigned a cost function
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION_FUNCTOR_H_ */
