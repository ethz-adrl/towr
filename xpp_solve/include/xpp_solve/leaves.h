/**
 @file    leafs.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 6, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_XPP_SOLVE_INCLUDE_XPP_SOLVE_LEAVES_H_
#define XPP_OPT_XPP_SOLVE_INCLUDE_XPP_SOLVE_LEAVES_H_

#include "composite.h"

namespace opt {

/** @brief An specific constraint implementing the above interface.
  *
  * Classes that derive from this represent the actual "meat".
  * But somehow also just a Composite of OptimizationVariables.
  */
class Constraint : public Component {
public:
  using VariablesPtr = Composite::Ptr;

  Constraint(const VariablesPtr& variables, int row_count, const std::string& name);
  virtual ~Constraint() {};

  Jacobian GetJacobian() const override;


protected:
  const VariablesPtr GetVariables() const { return variables_; };

private:
  /** @brief Jacobian of the component with respect to each decision variable set.
    */
  virtual void FillJacobianBlock(std::string var_set, Jacobian& jacobian_block) const = 0;
  VariablesPtr variables_;

  // not neccessary for constraints or costs
  virtual void SetValues(const VectorXd& x) override { assert(false); };
};




class Cost : public Constraint {
public:
  Cost(const VariablesPtr& variables, const std::string& name);
  virtual ~Cost() {};

  virtual VecBound GetBounds() const override
  {
    return VecBound(GetRows(), NoBound);
  };
};




class Variables : public Component {
public:
  Variables(int n_rows, const std::string name) : Component(n_rows, name) {};
  virtual ~Variables() {};

  // doesn't exist for optimization variables
  virtual Jacobian GetJacobian() const override { assert(false); };
};


// use this for placeholder if number of constraints not clear in c'tor.
// not recommended, requires SetRows() call.
static const int kSpecifyLater = -1;




} /* namespace opt */

#endif /* XPP_OPT_XPP_SOLVE_INCLUDE_XPP_SOLVE_LEAVES_H_ */
