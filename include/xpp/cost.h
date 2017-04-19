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

#include <xpp/opt/constraints/constraint.h>

namespace xpp {
namespace opt {

/** @brief Common interface to define a cost, which simply returns a scalar value
  */
// zmp_ specialization of constraint with just one row/constraint!
class Cost : public Constraint {
public:
  using VectorXd   = Eigen::VectorXd;
  using Jacobian   = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  Cost ();
  virtual ~Cost ();

  // both vector and jacobian only have 1 row
  VectorXd GetConstraintValues () const override;
  Jacobian GetConstraintJacobian() const override;
  VecBound GetBounds() const override { assert(false); /* costs don't have bounds */ };

  void SetWeight(double weight);

protected:
  virtual double GetCost () const = 0;
  virtual Jacobian GetJacobian() const = 0;

private:
  double weight_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_COST_H_ */
