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

#ifndef TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_

#include <string>
#include <vector>

#include <ifopt/constraint_set.h>

namespace towr {

/**
 * @brief Constraints evaluated at discretized times along a trajectory.
 *
 * Often one want to check the values of a specific constraint, e.g.
 * @ref RangeOfMotion, or @ref DynamicConstraint at specific times t along
 * the trajectory. This class is responsible for building the overall
 * Jacobian from the individual Jacobians at each time instance.
 *
 * @ingroup Constraints
 */
class TimeDiscretizationConstraint : public ifopt::ConstraintSet {
public:
  using VecTimes = std::vector<double>;
  using Bounds   = ifopt::Bounds;

  /**
   * @brief Constructs a constraint for ifopt.
   * @param T  The total duration of the trajectory.
   * @param dt The discretization interval at which each constraint is evaluated.
   * @param constraint_name  The name of the constraint.
   */
  TimeDiscretizationConstraint (double T, double dt, std::string constraint_name);

  /**
   * @brief construct a constraint for ifopt.
   * @param dts  Time instances at which to evaluate the constraints.
   * @param name The name of the constraint.
   */
  TimeDiscretizationConstraint (const VecTimes& dts, std::string name);
  virtual ~TimeDiscretizationConstraint () = default;

  Eigen::VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

protected:
  int GetNumberOfNodes() const;
  VecTimes dts_; ///< times at which the constraint is evaluated.

private:
  /**
   * @brief Sets the constraint value a specific time t, corresponding to node k.
   * @param t  The time along the trajectory to set the constraint.
   * @param k  The index of the time t, so t=k*dt
   * @param[in/out] g  The complete vector of constraint values, for which the
   *                   corresponding row must be filled.
   */
  virtual void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const = 0;

  /**
   * @brief Sets upper/lower bound a specific time t, corresponding to node k.
   * @param t  The time along the trajectory to set the bounds.
   * @param k  The index of the time t, so t=k*dt
   * @param[in/out] b The complete vector of bounds, for which the corresponding
   *                  row must be set.
   */
  virtual void UpdateBoundsAtInstance(double t, int k, VecBound& b) const = 0;

  /**
   * @brief Sets Jacobian rows at a specific time t, corresponding to node k.
   * @param t  The time along the trajcetory to set the bounds.
   * @param k  The index of the time t, so t=k*dt
   * @param var_set The name of the ifopt variables currently being queried for.
   * @param[in/out] jac  The complete Jacobian, for which the corresponding
   *                     row and columns must be set.
   */
  virtual void UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                        Jacobian& jac) const = 0;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_ */
