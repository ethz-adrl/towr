/**
 @file    a_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_

#include "optimization_variables.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>

namespace xpp {
namespace opt {

// zmp_ possibly remove the "A"
class AConstraint {
public:
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  AConstraint ();
  virtual ~AConstraint ();

  virtual void UpdateVariables(const OptimizationVariables*) = 0;

  /** A constraint always delivers a vector of constraint violations.
   */
  virtual VectorXd EvaluateConstraint () const = 0;

  /** The Jacobian of the constraints with respect to each decision variable set
    */
  virtual Jacobian GetJacobianWithRespectTo (std::string var_set) const = 0;

  /** For each returned constraint an upper and lower bound is given.
   */
  virtual VecBound GetBounds () const = 0;

  int GetNumberOfConstraints() const;

  void PrintStatus(double tol) const;

protected:
  std::string name_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_ */
