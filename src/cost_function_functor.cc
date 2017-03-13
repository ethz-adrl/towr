/**
 @file    cost_function_functor.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 3, 2016
 @brief   Brief description
 */

#include <xpp/cost_function_functor.h>

namespace xpp {
namespace opt {

CostFunctionFunctor::CostFunctionFunctor ()
{
  subject_        = nullptr;
  cost_container_ = nullptr;
}

CostFunctionFunctor::~CostFunctionFunctor ()
{
  // TODO Auto-generated destructor stub
}

void
CostFunctionFunctor::AddCosts (OptimizationVariables& subject,
                               CostContainer& costs)
{
  subject_        = &subject;
  cost_container_ = &costs;
  Base::set_inputs(subject.GetOptimizationVariableCount());
  Base::set_values(1); // cost function always returns a scalar value

  costs_added_    = true;
}

int
CostFunctionFunctor::operator () (const InputType& x,
                                  ValueType& obj_value) const
{
  assert(costs_added_ && inputs()!= 0 && values()!=0);
  subject_->SetAllVariables(x);
  obj_value(0) = cost_container_->EvaluateTotalCost();
  return 1;
}

} /* namespace zmp */
} /* namespace xpp */
