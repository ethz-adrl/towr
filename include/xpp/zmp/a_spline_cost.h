/**
 @file    a_spline_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Declaration of ASplineCost, QuadraticSplineCost, SquaredSplineCost
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_SPLINE_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_SPLINE_COST_H_

#include <xpp/zmp/a_cost.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace zmp {

/** @brief Calculates the scalar cost associated to spline coefficients.
  *
  * This class is responsible for getting the current value of the optimization
  * variables from the subject and calculating the scalar cost from these.
  */
class ASplineCost : public ACost {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef xpp::utils::MatVec MatVec;

  ASplineCost ();
  virtual ~ASplineCost ()  {}

  /** @brief Defines the matrices and vectors that when multiplied by the
    * spline coefficients determine the cost.
    */
  void Init(const MatVec&);
  void UpdateVariables(const OptimizationVariables*) override;

protected:
  VectorXd spline_coeff_; ///< the current spline coefficients
  MatVec matrix_vector_;  ///< a matrix and a vector used to calculate a scalar cost
};


class QuadraticSplineCost : public ASplineCost {

  /**  The cost is calculated as
    *  cost = x^T * M * x   +   v^T * x
    */
  double EvaluateCost () const override;
  virtual VectorXd EvaluateGradientWrt(std::string var_set) final;
};

class SquaredSplineCost : public ASplineCost {

  /**  The cost is calculated as:
    *  g = M*x + v
    *  cost = sqrt(g^T*g)
    */
  double EvaluateCost () const override;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_SPLINE_COST_H_ */
