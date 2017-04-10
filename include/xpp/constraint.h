/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_

#include <Eigen/Sparse> // for jacobians
#include <memory>

#include "optimization_variables.h"
#include "optimization_variables_container.h"

namespace xpp {
namespace opt {

/** Common interface providing constraint values and bounds.
  */
class Constraint {
public:
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using ParametrizationPtr = std::shared_ptr<OptimizationVariables>;
  using JacobianNamed = std::pair<std::string, Jacobian>;

  Constraint ();
  virtual ~Constraint ();

  /** @brief Jacobian of the constraints with respect to each decision variable set
    */
  Jacobian GetJacobianWithRespectTo (std::string var_set) const;

  /** @brief A constraint always delivers a vector of constraint violations.
   */
  VectorXd GetConstraintValues() const;

  /** @brief For each returned constraint an upper and lower bound is given.
    */
  VecBound GetBounds();

  void PrintStatus(double tol) const;
  int GetNumberOfConstraints() const;

  /** @brief Implement this if the Jacobians change with different values of the
    * optimization variables, so are not constant.
    */
  virtual void UpdateJacobians() {/* do nothing assuming Jacobians constant */};

  /** @brief A constraint always delivers a vector of constraint violations.
    *
    * This is specific to each type of constraint and must be implemented
    * by the user.
    */
  virtual void UpdateConstraintValues () = 0;

protected:
  /** @brief Determines the size of constraints, bounds and jacobians.
    */
  void SetDimensions(const std::vector<ParametrizationPtr>&, int num_constraints);

  /** @returns a writable reference to modify the Jacobian of the constraint.
    *
    * @param var_set The differentiation of the constraint w.r.t these variables
    *                produces this Jacobian.
   */
  Jacobian& GetJacobianRefWithRespectTo (std::string var_set);

  std::string name_;
  VectorXd g_;
  VecBound bounds_;

private:

  /** @brief For each returned constraint an upper and lower bound is given.
    *
    * This is specific to each type of constraint and must be implemented
    * by the user.
    */
  virtual void UpdateBounds () = 0;


  std::vector<JacobianNamed> jacobians_;
  int num_constraints_ = 0;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_ */
