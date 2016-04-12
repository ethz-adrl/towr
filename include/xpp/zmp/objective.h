/*
 * cost_function.h
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBJECTIVE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBJECTIVE_H_

#include <xpp/zmp/cost_function.h>

namespace xpp {
namespace zmp {


/**
 * Implements the higher level value and derivative calculation.
 */
class Objective {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::zmp::CostFunction CostFunction;
  typedef Eigen::NumericalDiff<CostFunction> JacCostFunction;

public:
  explicit Objective (const CostFunction& cost_function_functor);
  virtual ~Objective () {};

  double EvalObjective(const VectorXd& x_coeff) const;
  Eigen::VectorXd EvalGradientOfObjectiveNumeric(const VectorXd& x_coeff) const;
  Eigen::VectorXd EvalGradientOfObjectiveAnalytic(const VectorXd& x_coeff) const;


private:
  CostFunction cost_function_;
  JacCostFunction jac_cost_function_;

};




} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBJECTIVE_H_ */
