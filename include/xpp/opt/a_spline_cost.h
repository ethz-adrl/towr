/**
 @file    a_spline_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Declaration of ASplineCost, QuadraticSplineCost, SquaredSplineCost
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SPLINE_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SPLINE_COST_H_

#include <xpp/matrix_vector.h>
#include <xpp/cost.h>

namespace xpp {
namespace opt {

class BaseMotion;

/** @brief Calculates the scalar cost associated to spline coefficients.
  *
  * This class is responsible for getting the current value of the optimization
  * variables from the subject and calculating the scalar cost from these.
  */
class ASplineCost : public Cost {
public:
  using VectorXd = Eigen::VectorXd;
  using BaseMotionPtrS = std::shared_ptr<BaseMotion>;

  ASplineCost (const OptVarsPtr&);
  virtual ~ASplineCost (){};

  void Update() override { /*com_motion_ always up-to-date*/ };

protected:
  BaseMotionPtrS com_motion_;
  MatVec matrix_vector_;  ///< a matrix and a vector used to calculate a scalar cost
};


class QuadraticSplineCost : public ASplineCost {
public:
  QuadraticSplineCost(const OptVarsPtr&, const MatVec&);
  virtual ~QuadraticSplineCost();

private:
  /**  The cost is calculated as
    *  cost = x^T * M * x   +   v^T * x
    */
  double EvaluateCost () const override;
  virtual VectorXd EvaluateGradientWrt(std::string var_set) final;
};

//class SquaredSplineCost : public ASplineCost {
//
//  /**  The cost is calculated as:
//    *  g = M*x + v
//    *  cost = sqrt(g^T*g)
//    */
//  double EvaluateCost () const override;
//  virtual VectorXd EvaluateGradientWrt(std::string var_set) final { assert(false); };
//
//};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SPLINE_COST_H_ */
