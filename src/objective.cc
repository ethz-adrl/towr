/*
 * cost_funtion.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#include <xpp/zmp/objective.h>

#include <array>

namespace xpp {
namespace zmp {



Objective::Objective (const CostFunction& cost_function_functor)
    :cost_function_(cost_function_functor),
     jac_cost_function_(cost_function_functor)
{}


double
Objective::EvalObjective(const Eigen::VectorXd& x) const
{
  return cost_function_.EvalCostFunction(x);
}


Eigen::VectorXd
Objective::EvalGradientOfObjectiveNumeric(const Eigen::VectorXd& x) const
{
  typedef Eigen::MatrixXd JacobianType;
  JacobianType jac = JacobianType(jac_cost_function_.values(), jac_cost_function_.inputs());
  jac_cost_function_.df(x,jac);
  return jac.transpose();
}


Eigen::VectorXd
Objective::EvalGradientOfObjectiveAnalytic(const Eigen::VectorXd& x) const
{
  return cost_function_.cf_.M*x;
}




} /* namespace zmp */
} /* namespace xpp */
