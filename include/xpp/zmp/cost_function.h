/*
 * cost_function1.h
 *
 *  Created on: Apr 12, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION1_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION1_H_

#include <xpp/utils/eigen_num_diff_functor.h>
#include <xpp/zmp/problem_specification.h>
#include <xpp/zmp/nlp_structure.h>

namespace xpp {
namespace zmp {

/**
 * Implements only the cost function calculation, none of the derivatives.
 */
class CostFunction : public xpp::utils::EigenNumDiffFunctor<double>,
                     public ProblemSpecification {
public:
  typedef xpp::utils::EigenNumDiffFunctor<double> EigenNumDiffFunctord;
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::zmp::ProblemSpecification::SupportPolygonContainer SupportPolygonContainer;
  typedef xpp::zmp::NlpStructure::StdVecEigen2d StdVecEigen2d;
  typedef Eigen::VectorXd VectorXd;
  typedef Eigen::Vector2d Vector2d;

  explicit CostFunction(const ContinuousSplineContainer& spline_structure,
                        const SupportPolygonContainer& supp_polygon_container,
                        const NlpStructure& nlp_structure);

public:
  /**
   * This implements the value of the cost function later used by Eigen::NumDiff
   *
   * @param x_coeff the inputs to the function
   * @param obj_value the one-dimensional output (obj_value(0)) of the cost function
   */
  int operator() (const InputType& x, ValueType& obj_value) const
  {
    obj_value(0) = EvalCostFunction(x);
    return 1;
  }
  double EvalCostFunction(const InputType& x) const;

  MatVec cf_;
  NlpStructure nlp_structure_;
  static MatVec CreateMinAccCostFunction(const ContinuousSplineContainer& spline_structure);
private:
  double MinimizeAcceleration(const VectorXd& x_coeff) const;
  double PenalizeFootholdFromPlanned(const StdVecEigen2d& footholds) const;
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FUNCTION1_H_ */
