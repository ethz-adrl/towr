/**
 @file    a_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "variable_names.h"

namespace xpp {
namespace opt {

class OptimizationVariables;

class AConstraint {
public:
  struct Bound {
    Bound(double lower = 0.0, double upper = 0.0) {
      lower_ = lower;
      upper_ = upper;
    }
    double lower_;
    double upper_;

    void operator-=(double scalar) {
      lower_ -= scalar;
      upper_ -= scalar;
    }
  };
  typedef Eigen::VectorXd VectorXd;
  typedef std::vector<Bound> VecBound;
  typedef Eigen::SparseMatrix<double, Eigen::RowMajor> Jacobian;

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

  static const Bound kNoBound_;
  static const Bound kEqualityBound_;
  static const Bound kInequalityBoundPositive_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_CONSTRAINT_H_ */
