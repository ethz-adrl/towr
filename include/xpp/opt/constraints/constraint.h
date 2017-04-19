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
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/bound.h>
#include <xpp/optimization_variables.h>
#include <xpp/optimization_variables_container.h>

namespace xpp {
namespace opt {


/** Common interface providing constraint values and bounds.
  */
// zmp_ rename according to composite patterns (base, leaf, composite).
class ConstraintBase {
public:
  using VectorXd   = Eigen::VectorXd;
  using Jacobian   = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using OptVarsPtr = std::shared_ptr<OptimizationVariablesContainer>;

  ConstraintBase ();
  virtual ~ConstraintBase ();

  /** @brief A constraint always delivers a vector of constraint violations.
   */
  virtual VectorXd GetConstraintValues() const = 0;

  /** @brief For each returned constraint an upper and lower bound is given.
    */
  virtual VecBound GetBounds() const = 0;
  virtual Jacobian GetConstraintJacobian() const = 0;

  int GetNumberOfConstraints() const;
//  int GetNumberOfOptVariables() const;

protected:
  /** @brief Determines the size of constraints, bounds and jacobians.
    */
  int num_constraints_ = 0;
};


class Constraint : public ConstraintBase {
public:

  Jacobian GetConstraintJacobian() const override;

protected:
  void SetDimensions(const OptVarsPtr&, int num_constraints); // zmp_ incorporate this into constructor

private:
  OptVarsPtr opt_vars_;
  /** @brief Jacobian of the constraints with respect to each decision variable set.
    */
  virtual void FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const = 0;
};



} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_ */
