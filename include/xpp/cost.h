/**
 @file    cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a cost for the NLP problem.
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_COST_H_
#define XPP_OPT_INCLUDE_XPP_OPT_COST_H_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "optimization_variables_container.h"

namespace xpp {
namespace opt {

/** @brief Common interface to define a cost, which simply returns a scalar value
  */
// zmp_ specialization of constraint with just one row?
class Cost {
public:
  using VectorXd = Eigen::VectorXd;
  using OptVarsPtr = std::shared_ptr<OptimizationVariablesContainer>;

  Cost (const OptVarsPtr& opt_vars_container);
  virtual ~Cost ();

  double EvaluateWeightedCost () const;
  VectorXd EvaluateCompleteGradient();

  void SetWeight(double weight);
  int GetVariableCount() const;

protected:
  virtual double EvaluateCost () const = 0;
  virtual VectorXd EvaluateGradientWrt(std::string var_set) = 0;

private:
  double weight_;

  std::vector<std::string> all_variable_ids_;
  int n_variables_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_COST_H_ */
