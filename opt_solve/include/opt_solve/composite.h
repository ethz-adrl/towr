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
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "nlp_bound.h"

namespace opt {


/** @brief Interface representing either costs, constraints or opt variables.
 *
 * Every individual constraint (primitive) or composite of constraints follows
 * this interface and therefore they can be used interchangeably. Costs are
 * seen as constraints with just one row.
 *
 * "smallest common denominator" of variables, costs and constraints
 *
 * see https://sourcemaking.com/design_patterns/composite
 */
class Component {
public:
  using Ptr  = std::shared_ptr<Component>;
  using PtrU = std::unique_ptr<Component>;

  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using VectorXd = Eigen::VectorXd;
  using VecBound = std::vector<NLPBound>;

  Component(int num_rows, const std::string name);
  virtual ~Component() {};

  /** @returns A vector of values for current cost or constraints.
    *
    * For constraints, each row represents one constraint.
    * For a cost, the vector has dimension 1, is scalar.
    */
  virtual VectorXd GetValues() const = 0;

  /** @returns For each row an upper and lower bound is given.
    */
  virtual VecBound GetBounds() const = 0;


  // only needed for optimization variables
  virtual void SetValues(const VectorXd& x) = 0;

  /** @returns Derivatives of each row w.r.t each decision variable.
    *
    * For a constraint this is a matrix, one row per constraint.
    * For a cost only the first row is filled (gradient transpose).
    */
  // not needed for optimization variables
  virtual Jacobian GetJacobian() const = 0;


  /** @returns 1 for cost and >1 for however many constraints or opt-variables.
    */
  int GetRows() const;
  std::string GetName() const;

  virtual void Print() const;

  void SetRows(int num_rows);

protected:

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
  using Ptr = std::shared_ptr<Composite>;
  using ComponentVec = std::vector<Component::Ptr>;

  /** @brief Determines weather composite represents cost or constraints.
    *
    * Constraints append individual constraint values and jacobian rows
    * below one another, whereas costs are added to the same row (0) to
    * always remain a single row.
    */
  Composite(const std::string name, bool is_cost);
  virtual ~Composite() {};

  Component::Ptr GetComponent(std::string name) const;
  template<typename T> std::shared_ptr<T> GetComponent(const std::string& name) const;

  /** @brief Adds a component to this composite.
   */
  void AddComponent (const Component::Ptr&);
  void ClearComponents();

  VectorXd GetValues   () const override;
  Jacobian GetJacobian () const override;
  VecBound GetBounds   () const override;
  void SetValues(const VectorXd& x) override;

  void Print() const override;

  ComponentVec GetNonzeroComponents() const;

private:
  bool is_cost_;
  ComponentVec components_;

  // either deep copy or shallow copy of components_ must be chosen
  Composite(const Composite& that) = delete;
};



// implementation of template functions
template<typename T>
std::shared_ptr<T> Composite::GetComponent(const std::string& name) const
{
  Component::Ptr c = GetComponent(name);
  return std::dynamic_pointer_cast<T>(c);
}


} /* namespace opt */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COMPOSITE_H_ */
