/**
 @file    composite.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Represents constraints/costs using Composite Pattern.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_

#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/bound.h>

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

  Component(int num_rows, const std::string name = "");
  virtual ~Component() {};

  /** @returns A vector of values for current cost or constraints.
    *
    * For constraints, each row represents one constraint.
    * For a cost, the vector has dimension 1, is scalar.
    */
  virtual VectorXd GetValues() const = 0;

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
  Composite(const std::string name, bool append_components = true);
  virtual ~Composite() {};

  /** @brief Adds a component to this composite.
    */
  void AddComponent (const ComponentPtr&);
  void ClearComponents();
  ComponentVec GetComponents() const; // spring_clean_ this should be hidden from user
  ComponentPtr GetComponent(std::string name) const;
  int GetComponentCount() const;

  VectorXd GetValues   () const override;
  Jacobian GetJacobian () const override;
  VecBound GetBounds   () const override;
  void SetValues(const VectorXd& x) override;

  void Print() const override;

private:
  bool append_components_;
  ComponentVec components_;
};


/** @brief An specific constraint implementing the above interface.
  *
  * Classes that derive from this represent the actual "meat".
  */
// spring_clean_ this is also somehow a composite of opt_vars
class Primitive : public Component {
public:
  using OptVarsPtr = std::shared_ptr<Composite>;

  Primitive();
  virtual ~Primitive() {};

  Jacobian GetJacobian() const override;

protected:
  void AddComposite(const OptVarsPtr&);

private:
  /** @brief Jacobian of the component with respect to each decision variable set.
    */
  virtual void FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const = 0;
  OptVarsPtr opt_vars_;
};



} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_ */
