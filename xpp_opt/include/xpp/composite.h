/**
 @file    composite.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Represents constraints/costs using Composite Pattern.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_

#include <cassert>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <string>
#include <vector>

#include "nlp_bound.h"

namespace xpp {

// make sure all jacobians in code follow same storage order
using MatrixSXd   = Eigen::SparseMatrix<double, Eigen::RowMajor>;
using Jacobian    = MatrixSXd;
using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;
using VectorXd    = Eigen::VectorXd;

/** @brief Interface representing either costs, constraints or opt variables.
 *
 * Every individual constraint (primitive) or composite of constraints follows
 * this interface and therefore they can be used interchangeably. Costs are
 * seen as constraints with just one row.
 *
 * see https://sourcemaking.com/design_patterns/composite
 */
class Component {
public:
  Component(int num_rows, const std::string name = "");
  virtual ~Component() {};

  /** @returns A vector of values for current cost or constraints.
    *
    * For constraints, each row represents one constraint.
    * For a cost, the vector has dimension 1, is scalar.
    */
  virtual VectorXd GetValues() const { assert(false); };

  // only needed for optimization variables
  virtual void SetValues(const VectorXd& x) { assert(false); };

  /** @returns Derivatives of each row w.r.t each decision variable.
    *
    * For a constraint this is a matrix, one row per constraint.
    * For a cost only the first row is filled (gradient transpose).
    */
  // not needed for optimization variables
  virtual Jacobian GetJacobian() const { assert(false); };

  /** @returns For each row an upper and lower bound is given.
    */
  virtual VecBound GetBounds() const { return VecBound(GetRows(), kNoBound_); };

  /** @returns 1 for cost and >1 for however many constraints or opt-variables.
    */
  int GetRows() const;
  std::string GetName() const;

  virtual void Print() const;

protected:
  void SetRows(int num_rows);
  void SetName(const std::string&);

private:
  int num_rows_ = 0;
  std::string name_;
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
  using ComponentVec = std::vector<ComponentPtr>;

  /** @brief Determines weather composite represents cost or constraints.
    *
    * Constraints append individual constraint values and jacobian rows
    * below one another, whereas costs are added to the same row (0) to
    * always remain a single row.
    *
    * Default (true) represent constraints.
    */
  Composite(const std::string name, bool append_components);
  virtual ~Composite() {};

  ComponentPtr GetComponent(std::string name) const;

  template<typename T>
  std::shared_ptr<T> GetComponent(const std::string& name) const
  {
    ComponentPtr c = GetComponent(name);
    return std::dynamic_pointer_cast<T>(c);
  }


  /** @brief Adds a component to this composite.
   *
   * @param use     If false, then the composite will simply hold the pointer to
   *                this component, but will be ignored it in all operations
   *                except "GetComponent()".
   */
  void AddComponent (const ComponentPtr&, bool use=true);
  void ClearComponents();
  ComponentVec GetComponents() const;
  int GetComponentCount() const;

  VectorXd GetValues   () const override;
  Jacobian GetJacobian () const override;
  VecBound GetBounds   () const override;
  void SetValues(const VectorXd& x) override;

  void Print() const override;

private:
  bool append_components_;
  ComponentVec components_;
  ComponentVec components_fixed_; // these are not optimized over

  // either deep copy or shallow copy of components_ must be chosen
  Composite(const Composite& that) = delete;
};


/** @brief An specific constraint implementing the above interface.
  *
  * Classes that derive from this represent the actual "meat".
  * But somehow also just a Composite of OptimizationVariables.
  */
class Primitive;
using Constraint = Primitive;
using Cost       = Primitive;

class Primitive : public Component {
public:
  using OptVarsPtr = std::shared_ptr<Composite>;

  Primitive();
  virtual ~Primitive() {};

  Jacobian GetJacobian() const override;

protected:
  void AddOptimizationVariables(const OptVarsPtr&);
  const OptVarsPtr GetOptVars() const { return opt_vars_; };

private:
  /** @brief Jacobian of the component with respect to each decision variable set.
    */
  virtual void FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const = 0;
  OptVarsPtr opt_vars_;
};

} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_ */
