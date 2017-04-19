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
#include <Eigen/Sparse>

#include "optimization_variables_container.h"

namespace xpp {
namespace opt {

/** @brief Common interface to define a cost, which simply returns a scalar value
  */
// zmp_ specialization of constraint with just one row/constraint!
class Cost {
public:
  using VectorXd   = Eigen::VectorXd;
  using Jacobian   = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using OptVarsPtr = std::shared_ptr<OptimizationVariablesContainer>;

  Cost (const OptVarsPtr& opt_vars_container);
  virtual ~Cost ();

  VectorXd GetWeightedCost () const;
  Jacobian GetWeightedJacobian();

  void SetWeight(double weight);
  int GetVariableCount() const;

protected:
  virtual double GetCost () const = 0;
  virtual Jacobian GetJacobian() const = 0;
  OptVarsPtr opt_vars_;

private:
  double weight_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_COST_H_ */
