/**
 @file    cost_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONTAINER_H_

#include <xpp/zmp/a_cost.h>
#include <xpp/utils/eigen_num_diff_functor.h>
#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

class CostContainer : public xpp::utils::EigenNumDiffFunctor<double>{
public:
  CostContainer (OptimizationVariables& subject);
  virtual ~CostContainer () {};

  void AddCost(const ACost& cost);
  double EvaluateTotalCost () const;

  int operator() (const InputType& x, ValueType& obj_value) const
  {
    subject_->SetVariables(x);
    obj_value(0) = EvaluateTotalCost();
    return 1;
  }

private:
  std::vector<const ACost*> costs_;
  OptimizationVariables* subject_;

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONTAINER_H_ */
