/**
 @file    composite.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Represents constraints/costs using Composite Pattern.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_

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

/** @brief Interface representing either costs or constraints.
 *
 * Every individual constraint (primitive) or composite of constraints follows
 * this interface and therefore they can be used interchangeably. Costs are
 * seen as constraints with just one row.
 *
 * see https://sourcemaking.com/design_patterns/composite
 */
class Component {
public:
  using VectorXd = Eigen::VectorXd;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  Component();
  virtual ~Component() {};

  /** @returns A component delivers a vector of values.
    *
    * For constraints, each row represents one constraint.
    * For a cost, the vector has dimension 1, is scalar.
    */
  virtual VectorXd GetValues() const = 0;

  /** @returns Derivatives of each row w.r.t each decision variable.
    *
    * For a constraint this is a matrix, one row per constraint.
    * For a cost only the first row is filled (gradient transpose).
    */
  virtual Jacobian GetJacobian() const = 0;

  /** @returns For each constraint an upper and lower bound is given.
    */
  virtual VecBound GetBounds() const { assert(false); /* costs don't need to implement this */ };

  int GetRows() const;

protected:
  int num_rows_ = 1; // corresponds to number of constraints, default 1 for costs
};


/** @brief An specific constraint implementing the above interface.
  *
  * Classes that derive from this represent the actual "meat".
  */
class Primitive : public Component {
public:
  using OptVarsPtr = std::shared_ptr<OptimizationVariablesContainer>;

  virtual ~Primitive() {};

protected:
  /** @brief Determines the size of components, bounds and jacobians.
    */
  void SetDimensions(const OptVarsPtr&, int num_rows);

private:
  Jacobian GetJacobian() const override;

  /** @brief Jacobian of the component with respect to each decision variable set.
    */
  virtual void FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const = 0;
  OptVarsPtr opt_vars_;
};


/** @brief A collection of components that forwards every call to these.
  *
  * This class follows the component interface as well, but doesn't actually
  * do any evaluation, but only stitches together the results of the
  * components it is holding.
  */
class Composite : public Component {
public:
  using ComponentPtr = std::shared_ptr<Component>;

  /** @brief Determines weather composite represents cost or constraints.
    *
    * Constraints append individual constraint values and jacobian rows
    * below one another, whereas costs are added to the same row (0) to
    * always remain a single row.
    *
    * Default (true) represent constraints.
    */
  Composite(bool append_components = true);
  virtual ~Composite() {};

  /** @brief Adds a component to this collection.
    */
  void AddComponent (const ComponentPtr& constraint);

  VectorXd GetValues   () const override;
  Jacobian GetJacobian () const override;
  VecBound GetBounds   () const override;

private:
  bool append_components_;
  std::vector<ComponentPtr> components_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_ */
