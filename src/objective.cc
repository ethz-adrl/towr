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
Objective::EvalObjective(const Eigen::VectorXd& x_coeff) const
{
  return cost_function_.EvalObjective(x_coeff);
}


Eigen::VectorXd
Objective::EvalGradientOfObjectiveNumeric(const Eigen::VectorXd& x_coeff) const
{

  typedef Eigen::MatrixXd JacobianType;
  JacobianType jac = JacobianType(jac_cost_function_.values(), jac_cost_function_.inputs());
  jac_cost_function_.df(x_coeff,jac);
  return jac.transpose();
}


Eigen::VectorXd
Objective::EvalGradientOfObjectiveAnalytic(const Eigen::VectorXd& x_coeff) const
{
  return cost_function_.cf_.M*x_coeff;
}




} /* namespace zmp */
} /* namespace xpp */
