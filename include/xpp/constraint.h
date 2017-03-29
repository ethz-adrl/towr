/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Abstract class representing a constraint for the NLP problem.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_

#include "optimization_variables.h"
#include "parametrization.h"

#include <Eigen/Sparse> // for jacobians
#include <memory>

namespace xpp {
namespace opt {

/** Common interface providing constraint values and bounds.
  */
class Constraint {
public:
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using ParametrizationPtr = std::shared_ptr<Parametrization>;
  using VarPair = std::pair<ParametrizationPtr,Jacobian>;

  Constraint ();
  virtual ~Constraint ();

  /** @brief Sets the values stored in variables_ to the current NLP ones
    */
  virtual void UpdateVariables(const OptimizationVariables*) final;

  /** @brief Jacobian of the constraints with respect to each decision variable set
    */
  virtual Jacobian GetJacobianWithRespectTo (std::string var_set) const final;

  /** @brief A constraint always delivers a vector of constraint violations.
    *
    * This is specific to each type of constraint and must be implemented
    * by the user.
    */

  VectorXd GetConstraintValues() const;
  virtual void UpdateConstraintValues () = 0;

  /** @brief For each returned constraint an upper and lower bound is given.
    *
    * This is specific to each type of constraint and must be implemented
    * by the user.
    */
  virtual VecBound GetBounds () const = 0;

  void PrintStatus(double tol) const;
  int GetNumberOfConstraints() const;

protected:
  /** @brief The values of these variables influence the constraint.
    *
    * Subscribes to these values and keeps them up-to-date to be used to
    * calculate the constraints and bounds.
    */
  void SetDependentVariables(const std::vector<ParametrizationPtr>&,
                             int num_constraints);

  /** @returns a writable reference to modify the Jacobian of the constraint.
    *
    * @param var_set The differentiation of the constraint w.r.t these variables
    *                produces this Jacobian.
   */
  Jacobian& GetJacobianRefWithRespectTo (std::string var_set);

  std::string name_;
  int num_constraints_ = 0;
  mutable VectorXd g_;
  mutable VecBound bounds_;

private:
  /** @brief Implement this if the Jacobians change with different values of the
    * optimization variables, so are not constant.
    */
  virtual void UpdateJacobians() {/* do nothing assuming Jacobians constant */};
  std::vector<VarPair> variables_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_ */
