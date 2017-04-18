/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "bound.h"
#include "optimization_variables.h"
#include "optimization_variables_container.h"
#include "soft_constraint.h" // only for friend declaration

namespace xpp {
namespace opt {


/** Common interface providing constraint values and bounds.
  */
class Constraint {
public:
  using VectorXd      = Eigen::VectorXd;
  using Jacobian      = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using OptVarPtr     = std::shared_ptr<OptimizationVariables>;
  using JacobianNamed = std::pair<std::string, Jacobian>;
  using OptVarsPtr    = std::shared_ptr<OptimizationVariablesContainer>;

  Constraint ();
  virtual ~Constraint ();


  /** @brief A constraint always delivers a vector of constraint violations.
   */
  virtual VectorXd GetConstraintValues() const = 0;

  // zmp_ possibly make constant
  Jacobian GetConstraintJacobian();

  /** @brief For each returned constraint an upper and lower bound is given.
    */
  virtual VecBound GetBounds() const = 0;


//  void PrintStatus(double tol) const;
  int GetNumberOfConstraints() const;
  int GetNumberOfOptVariables() const;


  /** Updates members (constraints/jacobians) using newest opt. variables.
    */
  void Update();


protected:
  /** @brief Determines the size of constraints, bounds and jacobians.
    */
  void SetDimensions(const std::vector<OptVarPtr>&, int num_constraints);

  /** @returns a writable reference to modify the Jacobian of the constraint.
    *
    * @param var_set The differentiation of the constraint w.r.t these variables
    *                produces this Jacobian.
   */
  Jacobian& GetJacobianRefWithRespectTo (std::string var_set);

  std::string name_; // zmp_ possiby remove, only used for printouts
  int num_constraints_;

private:


  // zmp_ these values are only accessed by the soft constraint, refactor
  friend VectorXd SoftConstraint::EvaluateGradientWrt(std::string);
  /** @brief Jacobian of the constraints with respect to each decision variable set
    */
  Jacobian GetJacobianWithRespectTo (std::string var_set) const;

  /** @brief Implement this if the Jacobians change with different values of the
    * optimization variables, so are not constant.
    */
  // zmp_ not clear when this is called, make more functions private
  virtual void UpdateJacobians() {/* do nothing assuming Jacobians constant */};


  std::vector<JacobianNamed> jacobians_;
  Jacobian complete_jacobian_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_ */
