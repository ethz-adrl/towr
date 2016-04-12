/*
 * cost_function1.h
 *
 *  Created on: Apr 12, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION1_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION1_H_

#include <xpp/utils/eigen_num_diff_functor.h>
#include <xpp/utils/geometric_structs.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

/**
 * Implements only the cost function calculation, none of the derivatives.
 */
class CostFunction : public xpp::utils::EigenNumDiffFunctor<double> {
public:
  typedef EigenNumDiffFunctor<double> Base;
  typedef xpp::utils::MatVec MatVec;

  explicit CostFunction(const ContinuousSplineContainer& spline_structure);

public:
  /**
   * This implements the value of the cost function later used by Eigen::NumDiff
   *
   * @param x_coeff the inputs to the function
   * @param obj_value the one-dimensional output (obj_value(0)) of the cost function
   */
  int operator() (const InputType& x_coeff, ValueType& obj_value) const;

  double EvalObjective(const Eigen::VectorXd& x_coeff) const;
  MatVec cf_;
  static MatVec CreateMinAccCostFunction(const ContinuousSplineContainer& spline_structure);
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION1_H_ */
