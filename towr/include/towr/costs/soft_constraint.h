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

#ifndef TOWR_COSTS_SOFT_CONSTRAINT_H_
#define TOWR_COSTS_SOFT_CONSTRAINT_H_

#include <ifopt/cost_term.h>

namespace towr {

/**
 * @brief Converts a constraint to a cost by weighing the quadratic violations.
 *
 * Let constraint g(x) \in R^m with upper bound b_u and lower bound b_l.
 * Let
 *
 *     g'(x) = g(x) - 0.5(b_u+b_l) = g(x) - b
 *
 * And it's derivative
 *
 *     dg'(x)/dx = J(x).
 *
 * Define a cost as
 *
 *     c(x) = 0.5 * g'^T * W * g', where W = diag(w1,...,wm).
 *
 * Then the gradient of the cost is defined as:
 *
 *     dc(x)/dx = (g'(x)^T * W * J)^T = J^T * W * (g(x)-b).
 *
 * @ingroup Costs
 */
class SoftConstraint : public ifopt::Component {
public:
  using ConstraintPtr = Component::Ptr;

  /**
   * @brief Creates a soft constraint (=cost) from a hard constraint.
   * @param constraint  The fully initialized constraint.
   *
   * Weights are set to identity, so each constraint violation contributes
   * equally to the cost.
   */
  SoftConstraint (const ConstraintPtr& constraint);
  virtual ~SoftConstraint () = default;

private:
  ConstraintPtr constraint_;
  VectorXd W_; ///< weights how each constraint violation contributes to the cost.
  VectorXd b_; /// average value of each upper and lower bound.

  /**
   * @brief The quadratic constraint violation transformed to a cost.
   *
   * c(x) = 0.5 * (g-b)^T * W * (g-b)
   */
  VectorXd GetValues () const override;

  /**
   * @brief The row-vector of derivatives of the cost term.
   *
   * dc(x)/dx = J^T * W * (g-b)
   */
  Jacobian GetJacobian() const override;

  // doesn't exist for cost, generated run-time error when used
  VecBound GetBounds() const final { return VecBound(GetRows(), ifopt::NoBound); };
  void SetVariables(const VectorXd& x) final { assert(false); };
};

} /* namespace towr */

#endif /* TOWR_COSTS_SOFT_CONSTRAINT_H_ */
