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

#ifndef TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_

#include <ifopt/constraint_set.h>

#include <towr/variables/nodes_variables_phase_based.h>
#include <towr/terrain/height_map.h> // for friction cone

namespace towr {

/**
 * @brief Ensures foot force that is unilateral and inside friction cone.
 *
 * This class is responsible for constraining the endeffector xyz-forces to
 * only push into the terrain and additionally stay inside the friction cone
 * according to the current slope.
 *
 * In order to keep the constraint linear and simple for the solver to solve,
 * we approximate the friction cone by a 4-sided pyramid.
 *
 * Attention: Constraint is enforced only at the spline nodes. In between
 * violations of this constraint can occur.
 *
 * @ingroup Constraints
 */
class ForceConstraint : public ifopt::ConstraintSet {
public:
  using Vector3d = Eigen::Vector3d;
  using EE = uint;

  /**
   * @brief Constructs a force contraint.
   * @param terrain  The gradient information of the terrain for friction cone.
   * @param force_limit_in_normal_direction  Maximum pushing force [N].
   * @param endeffector_id Which endeffector force should be constrained.
   */
  ForceConstraint (const HeightMap::Ptr& terrain,
                   double force_limit_in_normal_direction,
                   EE endeffector_id);
  virtual ~ForceConstraint () = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  NodesVariablesPhaseBased::Ptr ee_force_;  ///< the current xyz foot forces.
  NodesVariablesPhaseBased::Ptr ee_motion_; ///< the current xyz foot positions.

  HeightMap::Ptr terrain_; ///< gradient information at every position (x,y).
  double fn_max_;          ///< force limit in normal direction.
  double mu_;              ///< friction coeff between robot feet and terrain.
  int n_constraints_per_node_; ///< number of constraint for each node.
  EE ee_;                  ///< The endeffector force to be constrained.

  /**
   * The are those Hermite-nodes that shape the polynomial during the
   * stance phases, while all the others are already set to zero force (swing)
   **/
  std::vector<int> pure_stance_force_node_ids_;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_ */
