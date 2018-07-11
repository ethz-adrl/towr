/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_CONSTRAINTS_LINEAR_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_LINEAR_CONSTRAINT_H_

#include <ifopt/constraint_set.h>

namespace towr {

/**
 * @brief Calculates the constraint violations for linear constraints.
 *
 * @ingroup Constraints
 */
class LinearEqualityConstraint : public ifopt::ConstraintSet {
public:
  using MatrixXd = Eigen::MatrixXd;

  /**
   * @brief Defines the elements of the linear constraint as g = Mx+v = 0.
   *
   * @param M  The matrix M defining the slope.
   * @param v  The vector v defining the constanct offset.
   * @param variable_set  The name of the variables x.
   */
  LinearEqualityConstraint (const MatrixXd& M,
                            const VectorXd& v,
                            const std::string& variable_set);
  virtual ~LinearEqualityConstraint () = default;

  VectorXd GetValues() const final;
  VecBound GetBounds() const final;
  void FillJacobianBlock (std::string var_set, Jacobian&) const final;

private:
  MatrixXd M_;
  VectorXd v_;
  std::string variable_name_;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_LINEAR_CONSTRAINT_H_ */
