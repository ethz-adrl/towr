/**
 @file    constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Represents constraints using Composite Pattern.
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


/** @brief Interface providing constraint values, bounds and jacobians.
 *
 * Every individual constraint or composite of constraints follows this
 * interface and therefore they can be used interchangeably.
 *
 * see https://sourcemaking.com/design_patterns/composite
 */
class Constraint {
public:
  using VectorXd   = Eigen::VectorXd;
  using Jacobian   = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  virtual ~Constraint() {};

  /** @returns A constraint always delivers a vector of constraint violations.
   */
  virtual VectorXd GetConstraintValues() const = 0;

  /** @returns For each returned constraint an upper and lower bound is given.
    */
  virtual VecBound GetBounds() const = 0;

  /** @returns Derivatives of each constraint row w.r.t each decision variable.
    */
  virtual Jacobian GetConstraintJacobian() const = 0;

  int GetNumberOfConstraints() const;

protected:
  int num_constraints_ = 0;
};



/** @brief An specific constraint implementing the above interface.
  *
  * Classes that derive from this represent the actual "meat".
  */
class ConstraintLeaf : public Constraint {
public:
  using OptVarsPtr = std::shared_ptr<OptimizationVariablesContainer>;

  virtual ~ConstraintLeaf() {};

  Jacobian GetConstraintJacobian() const override;

protected:
  /** @brief Determines the size of constraints, bounds and jacobians.
    */
  void SetDimensions(const OptVarsPtr&, int num_constraints);

private:
  /** @brief Jacobian of the constraints with respect to each decision variable set.
    */
  virtual void FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const = 0;
  OptVarsPtr opt_vars_;
};



/** @brief A collection of constraints that forwards every call to these.
  *
  * This class follows the constraint interface as well, but doesn't actually
  * do any evaluation, but only stitches together the results of the
  * constraints it is holding.
  */
class ConstraintComposite : public Constraint {
public:
  using ConstraintPtr   = std::shared_ptr<Constraint>;
  using ConstraitPtrVec = std::vector<ConstraintPtr>;

  virtual ~ConstraintComposite() {};

  /** @brief Adds a constraint to this collection.
    */
  void AddConstraint (const ConstraintPtr& constraint);

  VectorXd GetConstraintValues () const override;
  Jacobian GetConstraintJacobian () const override;
  VecBound GetBounds () const override;

private:
  ConstraitPtrVec constraints_;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINT_H_ */
