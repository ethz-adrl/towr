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

#ifndef TOWR_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_

#include <towr/variables/spline_holder.h>
#include <towr/variables/spline.h>

#include "time_discretization_constraint.h"

namespace towr {

/**
 * @brief Keeps the 6D base motion in a specified range.
 *
 * In general this constraint should be avoided, since a similar affect can be
 * achieved with RangeOfMotionConstraint.
 *
 * @ingroup Constraints
 */
class BaseMotionConstraint : public TimeDiscretizationConstraint {
public:
  /**
   * @brief Links the base variables and sets hardcoded bounds on the state.
   * @param T  The total time of the optimization horizon.
   * @param dt The discretization interval of the constraints.
   * @param spline_holder  Holds pointers to the base variables.
   */
  BaseMotionConstraint (double T, double dt, const SplineHolder& spline_holder);
  virtual ~BaseMotionConstraint () = default;

  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;

private:
  NodeSpline::Ptr base_linear_;
  NodeSpline::Ptr base_angular_;

  VecBound node_bounds_;     ///< same bounds for each discretized node
  int GetRow (int node, int dim) const;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_ */
