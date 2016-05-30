/**
 @file    a_quadratic_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to implement quadratic cost functions x*M*x + b*x.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_QUADRATIC_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_QUADRATIC_COST_H_

#include <xpp/zmp/a_cost.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

/** @brief Calculates the scalar cost for a given quadratic equation.
  *
  * This class is responsible for getting the current value of the optimization
  * variables from the subject and calculating the scalar cost from these.
  */
class AQuadraticCost : public ACost, public IObserver {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::MatVec MatVec;

  AQuadraticCost (OptimizationVariables& subject);
  virtual ~AQuadraticCost ()  {}

  /** @brief Defines the matrices and vectors used for calcuting the quadratic cost.
    *
    * The cost is calculated as cost = x^T * M * x   +   v^T * x
   */
  void Init(const MatVec& quadratic_equation);
  void Update() override;
  double EvaluateCost () const override;

private:
  OptimizationVariables* subject_;
  VectorXd x_coeff_;                ///< the current spline coefficients
  MatVec quadratic_equation_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_QUADRATIC_COST_H_ */
