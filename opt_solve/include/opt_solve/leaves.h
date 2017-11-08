/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**
 * @file   leaves.h
 * @brief  Declares the classes Variables, Cost and Constraint.
 */

#ifndef OPT_SOLVE_INCLUDE_OPT_LEAVES_H_
#define OPT_SOLVE_INCLUDE_OPT_LEAVES_H_

#include "composite.h"


namespace opt {

/**
 * @brief  A container holding a set of related optimization variables.
 *
 * This is a single set of variables representing a single concept, e.g
 * "spline coefficients" or "step durations".
 *
 * @sa Component
 */
class Variable : public Component {
public:
  /**
   * @brief Creates a set of variables representing a single concept.
   * @param n_var  Number of variables.
   * @param name   What the variables represent to (e.g. "spline coefficients").
   */
  Variable(int n_var, const std::string& name) : Component(n_var, name) {};
  virtual ~Variable() {};

  // doesn't exist for variables, generated run-time error when used.
  virtual Jacobian GetJacobian() const override final { assert(false); };
};



/**
 * @brief A container holding a set of related constraints.
 *
 * This container holds constraints representing a single concept, e.g.
 * @c n constraints keeping a foot inside its range of motion. Each of the
 * @c n rows is given by:
 * lower_bound < g(x) < upper_bound
 *
 * @sa Component
 */
class Constraint : public Component {
public:
  using VariablesPtr = Composite::Ptr;

  /**
   * @brief Creates constraints on the variables @c x.
   * @param x  The variables that define the constraint.
   * @param n_constraints  The number of constraints.
   * @param name  What these constraints represent.
   */
  Constraint(const VariablesPtr& x, int n_constraints, const std::string& name);
  virtual ~Constraint() {};

  /**
   * @brief  The matrix of derivatives for these constraints and variables.
   *
   * Assuming @c n constraints and @c m variables, the returned Jacobian
   * has dimensions n x m. Every row represents the derivatives of a single
   * constraint, whereas every column refers to a single optimization variable.
   *
   * This function only combines the user-defined jacobians from
   * FillJacobianBlock().
   */
  Jacobian GetJacobian() const override final;

protected:
  /**
   * @brief Read access to the value of the optimization variables.
   *
   * This must be used to formulate the constraint violation and Jacobian.
   */
  const VariablesPtr GetVariables() const { return variables_; };

private:
  /**
   * @brief Set individual Jacobians corresponding to each decision variable set.
   * @param var_set  Set of variables the current Jacobian block belongs to.
   * @param jac_block  Columns of the overall Jacobian affected by var_set.
   *
   * A convenience function so the user does not have to worry about the
   * ordering of variable sets. All that is required is that the user knows
   * the internal ordering of variables in each individual set and provides
   * the Jacobian of the constraints w.r.t. this set (starting at column 0).
   * GetJacobian() then inserts these columns at the correct position in the
   * overall Jacobian.
   *
   * If the constraint doen't depend on a @c var_set, this function should
   * simply do nothing.
   */
  virtual void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const = 0;
  VariablesPtr variables_;

  // doesn't exist for constraints, generated run-time error when used
  virtual void SetVariables(const VectorXd& x) override final { assert(false); };
};



/**
 * @brief A container holding a single cost term.
 *
 * This container builds a scalar cost term from the values of the variables.
 * This can be seen as a constraint with only one row and no bounds.
 *
 * @sa Component
 */
class Cost : public Constraint {
public:
  Cost(const VariablesPtr& variables, const std::string& name);
  virtual ~Cost() {};

private:
  /**
   * @brief  Returns the scalar cost term calculated from the @c variables.
   */
  virtual double GetCost() const = 0;

public:
  /**
   * @brief  Wrapper function that converts double to Eigen::VectorXd.
   */
  virtual VectorXd GetValues() const override final;

  /**
   * @brief  Returns infinite bounds (e.g. no bounds).
   */
  virtual VecBound GetBounds() const override final;
};

} /* namespace opt */

#endif /* OPT_SOLVE_INCLUDE_OPT_LEAVES_H_ */
