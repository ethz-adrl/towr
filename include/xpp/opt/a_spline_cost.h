/**
 @file    a_spline_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Declaration of ASplineCost, QuadraticSplineCost, SquaredSplineCost
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SPLINE_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SPLINE_COST_H_

#include <xpp/matrix_vector.h>
#include "../cost.h"
#include "base_motion.h"

namespace xpp {
namespace opt {

/** @brief Calculates the scalar cost associated to spline coefficients.
  *
  * This class is responsible for getting the current value of the optimization
  * variables from the subject and calculating the scalar cost from these.
  */
class ASplineCost : public Cost {
public:
  using VectorXd = Eigen::VectorXd;
  using ComMotionPtrU = std::unique_ptr<BaseMotion>;

  ASplineCost ();
  virtual ~ASplineCost (){};

  /** @brief Defines the matrices and vectors that when multiplied by the
    * spline coefficients determine the cost.
    */
  void Init(const MatVec&, const BaseMotion&);
  void UpdateVariables(const OptimizationVariables*) override;

protected:

  ComMotionPtrU com_motion_;
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
  virtual VectorXd EvaluateGradientWrt(std::string var_set) final { assert(false); };

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SPLINE_COST_H_ */
