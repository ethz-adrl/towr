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

#ifndef TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_

#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/euler_converter.h>

#include <towr/models/dynamic_model.h>

#include "time_discretization_constraint.h"

namespace towr {

/**
 * @brief  Ensure that the optimized motion complies with the system dynamics.
 *
 * At specific time instances along the trajecetory, this class checks the
 * current value of the optimization variables and makes sure that the
 * current robot state (pos,vel) and forces match the current acceleration
 * defined by the system dynamics.
 *
 * The physics-based acceleration is influenced by the robot state as
 *
 *     xdd(t) = f(x(t), xd(t), f(t))
 *
 * The constraint of the optimization variables w is then:
 *
 *     g(t) = acc_spline(t) - xdd(t)
 *          = acc_spline(t) - xdd(x(t), xd(t), f(t))
 *          = acc_spline(w) - xdd(w)
 *
 * @ingroup Constraints
 */
class DynamicConstraint : public TimeDiscretizationConstraint {
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  /**
   * @brief  Construct a Dynamic constraint
   * @param model  The system dynamics to enforce (e.g. centroidal, LIP, ...)
   * @param T   The total duration of the optimization.
   * @param dt  the discretization intervall at which to enforce constraints.
   * @param spline_holder  A pointer to the current optimization variables.
   */
  DynamicConstraint (const DynamicModel::Ptr& model,
                     double T, double dt,
                     const SplineHolder& spline_holder);
  virtual ~DynamicConstraint () = default;

private:
  NodeSpline::Ptr base_linear_;   ///< lin. base pos/vel/acc in world frame
  EulerConverter base_angular_;        ///< angular base state
  std::vector<NodeSpline::Ptr> ee_forces_; ///< endeffector forces in world frame.
  std::vector<NodeSpline::Ptr> ee_motion_; ///< endeffector position in world frame.

  mutable DynamicModel::Ptr model_;    ///< the dynamic model (e.g. Centroidal)

  /**
   * @brief The row in the overall constraint for this evaluation time.
   * @param k The index of the constraint evaluation at t=k*dt.
   * @param dimension Which base acceleration dimension this constraint is for.
   * @return The constraint index in the overall dynamic constraint.
   */
  int GetRow(int k, Dim6D dimension) const;

  /**
   * @brief Updates the model with the current state and forces.
   * @param t Time at which to query the state and force splines.
   */
  void UpdateModel(double t) const;

  void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const override;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_ */
