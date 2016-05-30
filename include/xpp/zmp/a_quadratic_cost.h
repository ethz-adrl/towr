/**
 @file    a_linear_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_QUADRATIC_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_QUADRATIC_COST_H_

#include <xpp/zmp/a_cost.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

class AQuadraticCost : public ACost, public IObserver {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::MatVec MatVec;

  AQuadraticCost (OptimizationVariables& subject);
  virtual ~AQuadraticCost ()  {}

  void Init(const MatVec& linear_equation);
  void Update() override;
  double EvaluateCost () const override;

private:
  OptimizationVariables* subject_;
  VectorXd x_coeff_;                ///< the current spline coefficients
  MatVec linear_equation_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_QUADRATIC_COST_H_ */
