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
 @file    leafs.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 6, 2017
 @brief   Brief description
 */

#ifndef OPT_SOLVE_INCLUDE_OPT_LEAVES_H_
#define OPT_SOLVE_INCLUDE_OPT_LEAVES_H_

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

#endif /* OPT_SOLVE_INCLUDE_OPT_LEAVES_H_ */
