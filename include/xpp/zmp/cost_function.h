/*
 * cost_function.h
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

class CostFunction {
public:
  typedef xpp::utils::MatVec MatVec;

public:
  CostFunction() {};
  CostFunction (const ContinuousSplineContainer& spline_structure);
  virtual
  ~CostFunction () {};

  double EvalObjective(const Eigen::VectorXd& x) const;
  Eigen::VectorXd EvalGradientOfObjective(const Eigen::VectorXd& x) const;


  MatVec CreateMinAccCostFunction() const;
private:
  int n_variables_;
  ContinuousSplineContainer spline_structure_;
  MatVec cf_quadratic_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION_H_ */
