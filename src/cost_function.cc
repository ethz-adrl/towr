/*
 * cost_funtion.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/cost_function.h>

namespace xpp {
namespace zmp {

CostFunction::CostFunction (const MatVec& cf_quadratic)
{
  cf_quadratic_ = cf_quadratic;
  n_variables_ = cf_quadratic.v.rows();
}


double
CostFunction::EvalObjective(const Eigen::VectorXd& x) const
{
  // some aliases
  const Eigen::MatrixXd& M = cf_quadratic_.M;
  const Eigen::VectorXd& q = cf_quadratic_.v;

  assert(x.rows() == n_variables_);
  double obj_value = 0.0;
  obj_value = x.transpose() * M * x;
  obj_value += q.transpose() * x;

  return obj_value;
}


Eigen::VectorXd
CostFunction::EvalGradientOfObjective(const Eigen::VectorXd& x) const
{
  assert(x.rows() == n_variables_);
  return cf_quadratic_.M*x;
}

} /* namespace zmp */
} /* namespace xpp */
