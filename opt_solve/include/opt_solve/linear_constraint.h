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

#ifndef OPT_SOLVE_INCLUDE_OPT_LINEAR_EQUALITY_CONSTRAINT_H_
#define OPT_SOLVE_INCLUDE_OPT_LINEAR_EQUALITY_CONSTRAINT_H_

#include "leaves.h"

namespace opt {

/**
 * @brief Calculates the constraint violations for linear constraints.
 */
class LinearEqualityConstraint : public Constraint {
public:
  using MatrixXd = Eigen::MatrixXd;

  /**
   * @brief Defines the elements of the linear constraint as g = Mx+v = 0.
   *
   * @param x  The optimization variables x.
   * @param M  The matrix M defining the slope.
   * @param v  The vector v defining the constanct offset.
   * @param variable_set  The name of the variables x.
   */
  LinearEqualityConstraint (const VariablesPtr& x,
                            const MatrixXd& M,
                            const VectorXd& v,
                            const std::string& variable_set);
  virtual ~LinearEqualityConstraint ();

  VectorXd GetValues() const override final;
  VecBound GetBounds() const override final;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override final;

private:
  MatrixXd M_;
  VectorXd v_;
  std::string variable_name_;
};

} /* namespace opt */

#endif /* OPT_SOLVE_INCLUDE_OPT_LINEAR_EQUALITY_CONSTRAINT_H_ */
